import gdsfactory as gf
import numpy as np
from gdsfactory.typings import Layer
from gdsfactory.component import Component
from gdsfactory.path import Path, _fresnel, _rotate_points
from gdsfactory.cross_section import cross_section
from gdsfactory.generic_tech import get_generic_pdk
from gdsfactory.pdk import get_active_pdk
from gdsfactory.typings import Layer, LayerSpec, LayerSpecs
PDK = get_generic_pdk()
PDK.activate()
@gf.cell
def taper_rsoa(
    AngleRsoa:float = 13,
    WidthRsoa:float = 8,
    WidthWg:float = 0.8,
    LengthRsoa:float = 200,
    LengthAdd:float = 100,
    RadiusBend:float = 50,
    layer: LayerSpec = "WG",
    layers: LayerSpecs | None = None,
    Name = "taper_rsoa"
)->Component:
    c = gf.Component(Name)
    layers = layers or [layer]
    ebend = c << gf.components.bend_euler(angle=-AngleRsoa,width = WidthWg,radius = RadiusBend,layer = layer)
    rtaper = c << gf.components.taper(length=LengthRsoa, width1=WidthWg, width2=WidthRsoa,layer = layer)
    rstr = c << gf.components.straight(length=LengthAdd,width=WidthRsoa,layer = layer)
    rtaper.connect(port="o1",destination=ebend.ports["o2"])
    rstr.connect(port="o1",destination=rtaper.ports["o2"])
    c.add_port("o1", port=rstr.ports["o1"])
    c.add_port("o2", port=ebend.ports["o1"])
    return c

def cir2end(
        WidthNear:float = 1,
        WidthEnd:float = 0.5,
        LengthTaper:float = 100,
        Pitch:float=10,
        RadiusBend0:float = 50,
        Period:float = 5.5,
        layer: LayerSpec = "WG",
        layers: LayerSpecs | None = None,
        Name = "cir2end"
)->Component:
    c = gf.Component(Name)
    layers = layers or [layer]
    for layer in layers:
        layer = gf.get_layer(layer)
        taper = c << gf.c.taper(width1=WidthNear, width2=WidthEnd, length=LengthTaper, layer=layer)
        if RadiusBend0 - Period * Pitch < 10:
            Period = (2 * RadiusBend0-10) // Pitch / 2   # avoid minus radius
        #circle
        bendcir = list(range(int(2 * Period)))
        bendcir[0] = c << gf.c.bend_circular180(width=WidthEnd, radius=RadiusBend0)
        bendcir[0].connect("o1", destination=taper.ports["o2"])
        for i in range(int(2 * Period - 1)):
            bendcir[i + 1] = c << gf.c.bend_circular180(width=WidthEnd, radius=RadiusBend0 - (i + 1) * Pitch / 2)
            bendcir[i + 1].connect("o1", destination=bendcir[i].ports["o2"])
        # setports
        c.add_port(name="o1", port=taper.ports["o1"])
    return c
def euler_Bend_Half(
        radius: float = 10,
        angle: float = 90,
        p: float = 0.5,
        use_eff: bool = False,
        npoints: int | None = None,
) -> Path:
    """Returns an euler bend that adiabatically transitions from straight to curved.

    `radius` is the minimum radius of curvature of the bend.
    However, if `use_eff` is set to True, `radius` corresponds to the effective
    radius of curvature (making the curve a drop-in replacement for an arc).
    If p < 1.0, will create a "partial euler" curve as described in Vogelbacher et. al.
    https://dx.doi.org/10.1364/oe.27.031394

    Args:
        radius: minimum radius of curvature.
        angle: total angle of the curve.
        p: Proportion of the curve that is an Euler curve.
        use_eff: If False: `radius` is the minimum radius of curvature of the bend. \
                If True: The curve will be scaled such that the endpoints match an \
                arc with parameters `radius` and `angle`.
        npoints: Number of points used per 360 degrees.

    .. plot::
        p = euler_Bend_Half(radius=10, angle=45, p=1, use_eff=True, npoints=720)
        p.plot()

    """

    if (p <= 0) or (p > 1):
        raise ValueError("euler requires argument `p` be between 0 and 1")

    if angle < 0:
        mirror = True
        angle = np.abs(angle)
    else:
        mirror = False

    R0 = 1
    alpha = np.radians(angle * 2)
    Rp = R0 / (np.sqrt(p * alpha))
    sp = R0 * np.sqrt(p * alpha)
    s0 = 2 * sp + Rp * alpha * (1 - p)

    pdk = get_active_pdk()
    npoints = npoints or abs(int(angle / 360 * radius / pdk.bend_points_distance / 2))
    npoints = max(npoints, int(360 / angle) + 1)

    num_pts_euler = int(np.round(sp / (s0 / 2) * npoints))
    num_pts_arc = npoints - num_pts_euler

    # Ensure a minimum of 2 points for each euler/arc section
    if npoints <= 2:
        num_pts_euler = 0
        num_pts_arc = 2

    if num_pts_euler > 0:
        xbend1, ybend1 = _fresnel(R0, sp, num_pts_euler)
        xp, yp = xbend1[-1], ybend1[-1]
        dx = xp - Rp * np.sin(p * alpha / 2)
        dy = yp - Rp * (1 - np.cos(p * alpha / 2))
    else:
        xbend1 = ybend1 = np.asfarray([])
        dx = 0
        dy = 0

    s = np.linspace(sp, s0 / 2, num_pts_arc)
    xbend2 = Rp * np.sin((s - sp) / Rp + p * alpha / 2) + dx
    ybend2 = Rp * (1 - np.cos((s - sp) / Rp + p * alpha / 2)) + dy

    x = np.concatenate([xbend1, xbend2[1:]])
    y = np.concatenate([ybend1, ybend2[1:]])

    points1 = np.array([x, y]).T
    points2 = np.flipud(np.array([x, -y]).T)

    points2 = _rotate_points(points2, angle - 180)
    points2 += -points2[0, :]

    points = points2

    # Find y-axis intersection point to compute Reff
    start_angle = 180 * (angle < 0)
    end_angle = start_angle + angle
    dy = np.tan(np.radians(end_angle - 90)) * points[-1][0]
    Reff = points[-1][1] - dy
    Rmin = Rp

    # Fix degenerate condition at angle == 180
    if np.abs(180 - angle) < 1e-3:
        Reff = points[-1][1] / 2

    # Scale curve to either match Reff or Rmin
    scale = radius / Reff if use_eff else radius / Rmin
    points *= scale

    P = Path()

    # Manually add points & adjust start and end angles
    P.points = points
    P.start_angle = start_angle
    P.end_angle = end_angle
    P.info["Reff"] = Reff * scale
    P.info["Rmin"] = Rmin * scale
    if mirror:
        P.mirror((1, 0))
    return P

def RunRing_Pulley(
        WidthRing: float = 8,
        WidthNear:float = 5,
        LengthRun: float = 200,
        RadiusRing: float = 100,
        GapRing: float = 1,
        AngleCouple:float = 20,
        IsAD:bool = True,
        layer: LayerSpec = "WG",
        layers: LayerSpecs | None = None,
        Name:str = "RunRing_Pulley"
)->Component:
    c = gf.Component(Name)
    layers = layers or [layer]

    for layer in layers:
        layer = gf.get_layer(layer)

        secring = gf.Section(width=WidthRing, offset=0, layer=layer, port_names=("in", "out"))
        secnring = gf.Section(width=WidthNear, offset=0, layer=layer, port_names=("in", "out"))
        wgring = gf.CrossSection(sections=[secring])
        wgnear = gf.CrossSection(sections=[secnring])
        # run ring path
        rrun1 = gf.path.straight(length=LengthRun / 2)
        rring1 = gf.path.arc(radius=RadiusRing, angle=60)
        rring2 = gf.path.arc(radius=RadiusRing, angle=-60)
        rb1 = euler_Bend_Half(radius=RadiusRing, angle=30, p=0.5)
        rb2 = euler_Bend_Half(radius=RadiusRing, angle=-30, p=0.5)
        RingPath1 = rring1 + rb1 + rrun1
        RingPath2 = rring2 + rb2 + rrun1
        RP1 = c << gf.path.extrude(RingPath1, cross_section=wgring)
        RP2 = c << gf.path.extrude(RingPath2, cross_section=wgring)
        RP3 = c << gf.path.extrude(RingPath1, cross_section=wgring)
        RP4 = c << gf.path.extrude(RingPath2, cross_section=wgring)
        RP1.connect("out", destination=RP4.ports["out"])
        RP2.connect("in", destination=RP1.ports["in"])
        RP3.connect("out", destination=RP2.ports["out"])
        RP4.connect("in", destination=RP3.ports["in"])
        # out port
        r_delta = WidthRing / 2 + GapRing + WidthNear / 2
        rcoup1 = gf.path.arc(radius=RadiusRing + r_delta, angle=-AngleCouple / 2)
        rcoup2 = gf.path.arc(radius=RadiusRing + r_delta, angle=AngleCouple / 2)
        rcb1 = euler_Bend_Half(radius=RadiusRing + r_delta, angle=-AngleCouple / 2, p=0.5)
        rcb2 = euler_Bend_Half(radius=RadiusRing + r_delta, angle=AngleCouple / 2, p=0.5)
        RingCoup1 = rcoup1 + rcb2
        RingCoup2 = rcoup2 + rcb1
        # input through
        RC1 = c << gf.path.extrude(RingCoup1, cross_section=wgnear)
        RC2 = c << gf.path.extrude(RingCoup2, cross_section=wgnear)
        RC1.connect("in", destination=RP3.ports["in"])
        RC1.movey(r_delta)
        RC2.connect("in", destination=RC1.ports["in"])
        # ports:
        c.add_port(name="Input", port=RC1.ports["out"],orientation=0)
        c.add_port(name="Through", port=RC2.ports["out"])
        #add drop
        if IsAD:
            RC3 = c << gf.path.extrude(RingCoup1, cross_section=wgnear)
            RC4 = c << gf.path.extrude(RingCoup2, cross_section=wgnear)
            RC3.connect("in", destination=RP1.ports["in"])
            RC3.movey(-r_delta)
            RC4.connect("in", destination=RC3.ports["in"])
            c.add_port(name="Add", port=RC3.ports["out"],orientation=180)
            c.add_port(name="Drop", port=RC4.ports["out"])
        c.add_port(name="Rcen1",port=RP1.ports["out"])
        c.add_port(name="Rcen2", port=RP3.ports["out"])

    return c
def DoubleRunRing_pulley(
        WidthRing: float = 8,
        WidthNear: float = 5,
        WidthEnd: float=0.5,
        LengthRun: float = 200,
        LengthTaper:float=100,
        LengthR2R:float = 300,
        DeltaLength: float = 2,
        RadiusRing: float = 100,
        RadiusBend0:float=50,
        GapRing: float = 1,
        AngleCouple: float = 20,
        Pitch:float=10,
        Period:float=5.5,
        IsSameSide:bool=True,
        layer: LayerSpec = "WG",
        layers: LayerSpecs | None = None,
        Name = "DOubleRunRing_pulley"
)->Component:
    c = gf.Component(Name)
    layers = layers or [layer]
    for layer in layers:
        layer = gf.get_layer(layer)
        ring1 = c << RunRing_Pulley(WidthRing=WidthRing,
                                      WidthNear=WidthNear,
                                      GapRing=GapRing,
                                      LengthRun=LengthRun,
                                      RadiusRing=RadiusRing,
                                      AngleCouple=AngleCouple,
                                      layer=layer)
        ring2 = c << RunRing_Pulley(WidthRing=WidthRing,
                                      WidthNear=WidthNear,
                                      GapRing=GapRing,
                                      LengthRun=LengthRun+DeltaLength,
                                      RadiusRing=RadiusRing,
                                      AngleCouple=AngleCouple,
                                      layer=layer)
        str_r2r = c << gf.c.straight(width=WidthNear, length=LengthR2R, layer=layer)
        str_r2r.connect("o1", destination=ring1.ports["Drop"])
        ring2.connect("Drop", destination=str_r2r.ports["o2"])
        if IsSameSide:
            ring2.mirror_y(port_name="Drop")
        r2e1 = c << cir2end(WidthEnd=WidthEnd,
                              WidthNear=WidthNear,
                              LengthTaper=LengthTaper,
                              Pitch=Pitch,
                              RadiusBend0=RadiusBend0,
                              Period=Period,
                              layer=layer)
        r2e2 = c << cir2end(WidthEnd=WidthEnd,
                              WidthNear=WidthNear,
                              LengthTaper=LengthTaper,
                              Pitch=Pitch,
                              RadiusBend0=RadiusBend0,
                              Period=Period,
                              layer=layer)
        r2e1.connect("o1", destination=ring1.ports["Through"])
        r2e2.connect("o1", destination=ring1.ports["Add"])
        r2e2.mirror_y(port_name="o1")
        r2e3 = c << cir2end(WidthEnd=WidthEnd,
                              WidthNear=WidthNear,
                              LengthTaper=LengthTaper,
                              Pitch=Pitch,
                              RadiusBend0=RadiusBend0,
                              Period=Period,
                              layer=layer)
        r2e4 = c << cir2end(WidthEnd=WidthEnd,
                              WidthNear=WidthNear,
                              LengthTaper=LengthTaper,
                              Pitch=Pitch,
                              RadiusBend0=RadiusBend0,
                              Period=Period,
                              layer=layer)
        r2e3.connect("o1", destination=ring2.ports["Through"])
        r2e4.connect("o1", destination=ring2.ports["Add"])
        r2e3.mirror_y(port_name="o1")
        c.add_port(name="o1", port=ring1.ports["Input"])
        c.add_port(name="o2", port=ring2.ports["Input"])
        c.add_port(name="R1cen1",port=ring1.ports["Rcen1"])
        c.add_port(name="R1cen2", port=ring1.ports["Rcen2"])
        c.add_port(name="R2cen1",port=ring2.ports["Rcen1"])
        c.add_port(name="R2cen2", port=ring2.ports["Rcen2"])
    return c

def Coupler2X2(
        WidthSingle:float = 0.8,
        LengthCoup:float = 100,
        LengthBridge = 300,
        GapCoup = 1,
        layer: LayerSpec = "WG",
        layers: LayerSpecs | None = None,
        Name="Coupler2X2"
)->Component:
    c = gf.Component(Name)
    layers = layers or [layer]
    for layer in layers:
        layer = gf.get_layer(layer)
        DeltaC = GapCoup+WidthSingle
        #path
        coups = gf.Section(width = WidthSingle,offset=0,layer = layer,port_names=("in","out"))
        coupcs = gf.CrossSection(sections=[coups])
        couppath = gf.path.straight(length = LengthCoup)
        coupbridge =  gf.path.straight(length = LengthBridge)
        cbpath1 = gf.path.euler(radius=100,angle=30,use_eff = True)
        cbpath2 = gf.path.euler(radius=100,angle=-30,use_eff = True)
        coup1 = couppath+cbpath1+cbpath2+coupbridge
        coupb1 = cbpath2+cbpath1
        #coupler waveguide
        CW1 = c << gf.path.extrude(coup1,cross_section=coupcs)
        CW2 = c << gf.path.extrude(coup1,cross_section=coupcs)
        CW2.connect("out",destination=CW1.ports["in"])
        CW2.movey(-DeltaC)
        CW2.rotate(180,center=CW2.ports["out"])
        CB1 = c << gf.path.extrude(coupb1,cross_section=coupcs)
        Deltacb = CB1.get_ports_xsize()
        CBs1 = c << gf.c.straight(width = WidthSingle,length=Deltacb,layer = layer)
        CB2 = c << gf.path.extrude(coupb1,cross_section=coupcs)
        CBs2 = c << gf.c.straight(width = WidthSingle,length=Deltacb,layer = layer)
        CB1.connect("out",destination=CW1.ports["in"])
        CBs1.connect("o2",destination=CW2.ports["out"])
        CB2.connect("in",destination=CW2.ports["in"])
        CBs2.connect("o1",destination=CW1.ports["out"])
        #set port
        c.add_port(name="input1", port=CB1.ports["in"])
        c.add_port(name="input2", port=CBs1.ports["o1"])
        c.add_port(name="output2", port=CB2.ports["out"])
        c.add_port(name="output1", port=CBs2.ports["o2"])
        c.add_port(name="bridge1", port=CW1.ports["out"])
        c.add_port(name="bridge2", port=CW2.ports["out"])
    return c

def OpenPad(
        WidthOpen:float = 90,
        Enclosure:float = 10,
        openlayer:LayerSpec = (40,0),
        elelayer:LayerSpec = "M1",
        Name = "OpenPad"
)->Component:
    c = gf.Component(Name)
    pad = c << gf.c.straight(width = WidthOpen,length=WidthOpen,layer = openlayer)
    outpad = c << gf.geometry.offset(pad,distance=Enclosure,layer=elelayer)
    a_pad = WidthOpen
    #set ports
    c.add_port(name="left", port=pad.ports["o1"],center=[-Enclosure,0])
    c.add_port(name="right", port=pad.ports["o2"],center=[a_pad+Enclosure,0])
    c.add_port(name="up", port=pad.ports["o1"],orientation=90,center=[a_pad/2,a_pad/2+Enclosure])
    c.add_port(name="down", port=pad.ports["o1"], orientation=-90, center=[a_pad/2, -a_pad/2-Enclosure])
    return c
def DBR(
        Width1:float = 2,
        Width2:float = 1,
        WidthHeat = 4,
        WidthRoute = 10,
        Length1:float = 0.4,
        Length2:float = 0.5,
        Length1E:float = 0.6,
        Length2E:float = 0.7,
        Period:float = 100,
        IsSG: bool = False,
        IsHeat:bool = False,
        hetlayer:LayerSpec = (31,0),
        layer: LayerSpec = "WG",
        layers: LayerSpecs | None = None,
        Name = "DBR"
)->Component:
    c=gf.Component(Name)
    layers = layers or [layer]
    for layer in layers:
        op = gf.Component()
        r1 = op << gf.c.straight(length=Length1, width=Width1, layer=layer)
        r2 = op << gf.c.straight(length=Length2, width=Width2, layer=layer)
        r2.connect(port="o1", destination=r1.ports["o2"])
        op.add_port("o1",port=r1.ports["o1"])
        op.add_port("o2",port=r2.ports["o2"])
        if IsSG:
            deltap1 = (Length1E - Length1) / (Period - 1)
            deltap2 = (Length2E - Length2) / (Period - 1)
            XBegin = 0
            for i in range(Period):
                r1 = c << gf.c.straight(length=Length1+i*deltap1, width=Width1, layer=layer)
                r2 = c << gf.c.straight(length=Length2+i*deltap2, width=Width2, layer=layer)
                r1.movex(XBegin)
                r2.movex(XBegin+Length1+i*deltap1)
                XBegin +=Length1+i*deltap1+Length2+i*deltap2
                if not i:#if i == 0,the first
                    c.add_port(name="o1",port=r1.ports["o1"],orientation=180)
                elif i == Period-1:
                    c.add_port(name="o2",port=r2.ports["o2"],orientation=0)
        else:
            c.add_array(op,columns=Period,rows = 1,spacing=(Length2+Length1,100))
            c.add_port(name="o1",port=r1.ports["o1"])
            c.add_port(name="o2",port=r2.ports["o2"],center=[(Length1+Length2)*Period,0])
    if IsHeat:
        length_dbr = c.ports["o2"].center-c.ports["o1"].center
        heater = c << gf.c.straight(width=WidthHeat,length=length_dbr[0],layer = hetlayer)
        heater.connect("o1",c.ports["o1"]).rotate(180,"o1")
        heattaper1 = c << gf.c.taper(width1=WidthHeat,width2=WidthRoute,length=WidthRoute/2-WidthHeat/2,layer = hetlayer)
        heattaper2 = c << gf.c.taper(width1=WidthHeat, width2=WidthRoute, length=WidthRoute / 2 - WidthHeat / 2,
                                 layer=hetlayer)
        heattaper1.connect("o1",destination=heater.ports["o1"])
        heattaper2.connect("o1",destination=heater.ports["o2"])
        c.add_port(name="h1",port=heattaper1.ports["o2"])
        c.add_port(name="h2", port=heattaper2.ports["o2"])
    return c
def OffsetRamp(
        length: float = 10.0,
        width1: float = 5.0,
        width2: float | None = 8.0,
        offset: float = 0,
        layer: LayerSpec = "WG",
        Name="OffsetRamp"
)-> Component:
    """Return a offset ramp component.

    Based on phidl.

    Args:
        length: Length of the ramp section.
        width1: Width of the start of the ramp section.
        width2: Width of the end of the ramp section (defaults to width1).
        offset: Offset of the output center to input center
        layer: Specific layer to put polygon geometry on.
    """
    if width2 is None:
        width2 = width1
    xpts = [0, length, length, 0]
    ypts = [width1, width2/2+width1/2+offset, -width2/2+width1/2+offset, 0]
    c = Component(Name)
    c.add_polygon([xpts, ypts], layer=layer)
    c.add_port(
        name="o1", center=[0, width1 / 2], width=width1, orientation=180, layer=layer
    )
    c.add_port(
        name="o2",
        center=[length, width1 / 2+offset],
        width=width2,
        orientation=0,
        layer=layer,
    )
    return c
def GSGELE(
        WidthG:float = 80,
        WidthS:float = 25,
        GapGS:float = 6,
        LengthEle:float = 10000,
        IsPad:bool = False,
        PitchPad:float = 100,
        WidthOpen:float = 40,
        Enclosure:float = 10,
        openlayer:LayerSpec = open,
        elelayer:LayerSpec = "M2",
        Name = "GSGELE"
)->Component:
    c = gf.Component(Name)
    deltags = GapGS+WidthS/2+WidthG/2
    Greff = gf.c.straight(width = WidthG,length = LengthEle,layer = elelayer)
    Sreff = gf.c.straight(width = WidthS,length = LengthEle,layer = elelayer)
    S1 = c << Sreff
    G1 = c << Greff
    G2 = c << Greff
    G1.movey(deltags)
    G2.movey(-deltags)
    c.add_port(name="Oin1", port=S1.ports["o1"], center=[0, GapGS / 2 + WidthS / 2])
    c.add_port(name="Oout1", port=S1.ports["o2"], center=[0, GapGS / 2 + WidthS / 2])
    c.add_port(name="Oin2", port=S1.ports["o1"], center=[0, -GapGS / 2 - WidthS / 2])
    c.add_port(name="Oout2", port=S1.ports["o2"], center=[0, -GapGS / 2 - WidthS / 2])
    c.add_port(name="Gin1", port=G1.ports["o2"])
    c.add_port(name="Gout1", port=G1.ports["o1"])
    c.add_port(name="Sin", port=S1.ports["o2"])
    c.add_port(name="Sout", port=S1.ports["o1"])
    c.add_port(name="Gin2", port=G2.ports["o2"])
    c.add_port(name="Gout2", port=G2.ports["o1"])
    if IsPad:
        GSGPadarray = gf.Component()
        GSGPad1 = GSGPadarray << OpenPad(WidthOpen=WidthOpen,Enclosure=Enclosure, elelayer=elelayer, openlayer=openlayer)
        GSGPad2 = GSGPadarray << OpenPad(WidthOpen=WidthOpen,Enclosure=Enclosure, elelayer=elelayer, openlayer=openlayer)
        GSGPad3 = GSGPadarray << OpenPad(WidthOpen=WidthOpen,Enclosure=Enclosure, elelayer=elelayer, openlayer=openlayer)
        GSGPad2.movey(PitchPad)
        GSGPad3.movey(PitchPad*2)
        GSGPadarray.add_port("Pr1", port=GSGPad1.ports["right"])
        GSGPadarray.add_port("Pr2", port=GSGPad2.ports["right"])
        GSGPadarray.add_port("Pr3", port=GSGPad3.ports["right"])
        GPa = c << GSGPadarray
        GPa.connect("Pr1", destination=c.ports["Gin1"]).move([PitchPad/3, PitchPad/5])
        # Gin1
        pos_diff = GPa.ports["Pr1"].center - c.ports["Gin1"].center
        G2Pa1 = c << OffsetRamp(length=PitchPad/3, width1=WidthOpen+2*Enclosure, width2=80, offset=pos_diff[1], layer=elelayer)
        G2Pa1.connect("o2", destination=c.ports["Gin1"])
        # Sin
        pos_diff = GPa.ports["Pr2"].center - c.ports["Sin"].center
        G2Pa2 = c << OffsetRamp(length=PitchPad/3, width1=WidthOpen+2*Enclosure, width2=25, offset=pos_diff[1], layer=elelayer)
        G2Pa2.connect("o2", destination=c.ports["Sin"])
        # Gin2
        pos_diff = GPa.ports["Pr3"].center - c.ports["Gin2"].center
        G2Pa3 = c << OffsetRamp(length=PitchPad/3, width1=WidthOpen+2*Enclosure, width2=80, offset=pos_diff[1], layer=elelayer)
        G2Pa3.connect("o2", destination=c.ports["Gin2"])
    return c
