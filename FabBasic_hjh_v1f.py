#new:
#GSGELE: add "Is2Pad" key
import gdsfactory as gf
import numpy as np
import csv
from gdsfactory.typings import Layer
from gdsfactory.component import Component
from gdsfactory.path import Path, _fresnel, _rotate_points
from gdsfactory.typings import LayerSpec
from gdsfactory.cross_section import cross_section
from gdsfactory.generic_tech import get_generic_pdk
from gdsfactory.pdk import get_active_pdk
from gdsfactory.typings import Layer, LayerSpec, LayerSpecs ,Optional, Callable
from gdsfactory.technology import LayerLevel, LayerStack, LayerView, LayerViews
from gdsfactory.technology.layer_map import LayerMap
from pydantic import BaseModel
import FabDefine_YQF as FD
PDK = get_generic_pdk()
PDK.activate()
# layer define
LAYER = FD.LayerMapUserDef()

# %% add labels
'''add labels to optical ports'''
def add_labels_to_ports(
        component: Component,
        label_layer: LayerSpec = (1, 10),  # 指定文本标签的层次
        port_type: Optional[str] = "optical",
        **kwargs,
) -> Component:
    """Add labels to component ports.

    Args:
        component: to add labels.
        label_layer: layer spec for the label.
        port_type: to select ports.

    keyword Args:
        layer: select ports with GDS layer.
        prefix: select ports with prefix in port name.
        orientation: select ports with orientation in degrees.
        width: select ports with port width.
        layers_excluded: List of layers to exclude.
        port_type: select ports with port_type (optical, electrical, vertical_te).
        clockwise: if True, sort ports clockwise, False: counter-clockwise.
    """
    #new_component = component.copy()  # 创建组件的副本
    ports = component.get_ports_list(port_type=port_type, **kwargs)
    for port in ports:
        component.add_label(text=port.name, position=port.center, layer=label_layer)
    return component
def add_labels_decorator(func: Callable) -> Callable:
    def wrapper(*args, **kwargs) -> Component:
        component = func(*args, **kwargs)
        return add_labels_to_ports(component, **kwargs)
    return wrapper

# %% TaperRsoa
def Crossing_taper(
        WidthCross: float = 1,
        WidthWg: float = 0.45,
        LengthTaper: float = 100,
        oplayer: LayerSpec = "WG",
        Name="my_taper",
)->Component:
    Crossing = gf.Component(Name)
    center = Crossing << gf.c.straight(width = WidthWg,length = WidthWg,layer = oplayer)
    center.movex(-WidthWg/2)
    taper0 = gf.c.taper(width2 = WidthCross,width1 = WidthWg,layer = oplayer,length = LengthTaper)
    taper = list(range(4))
    for i in range(4):
        taper[i] = Crossing << taper0
        taper[i].connect("o2",destination=center.ports["o1"])
        #taper[i].move([WidthWg/2*np.cos(90*i),WidthWg/2*np.sin(90*i)])
        taper[i].rotate(-90*i)
        Crossing.add_port("o"+str(i+1),port=taper[i].ports["o1"])
    add_labels_to_ports(Crossing)
    return Crossing

def TaperRsoa(
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
# %% cir2end

def cir2end(
        WidthNear:float = 1,
        WidthEnd:float = 0.5,
        LengthTaper:float = 100,
        Pitch:float=10,
        RadiusBend0:float = 50,
        Period:float = 5.5,
        oplayer: LayerSpec = "WG",
        layers: LayerSpecs | None = None,
        Name = "cir2end"
)->Component:
    c = gf.Component(Name)
    taper = c << gf.c.taper(width1=WidthNear, width2=WidthEnd, length=LengthTaper, layer=oplayer)
    if RadiusBend0 - Period * Pitch < 10:
        Period = (2 * RadiusBend0-10) // Pitch / 2   # avoid minus radius
    #circle
    bendcir = list(range(int(2 * Period)))
    bendcir[0] = c << gf.c.bend_circular180(width=WidthEnd, radius=RadiusBend0,layer = oplayer)
    bendcir[0].connect("o1", destination=taper.ports["o2"])
    for i in range(int(2 * Period - 1)):
        bendcir[i + 1] = c << gf.c.bend_circular180(width=WidthEnd, radius=RadiusBend0 - (i + 1) * Pitch / 2,layer = oplayer)
        bendcir[i + 1].connect("o1", destination=bendcir[i].ports["o2"])
    # setports
    c.add_port(name="o1", port=taper.ports["o1"])
    return c
# %% euler_Bend_Half

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
# %% RingPulley
@gf.cell
def RingPulley(
        WidthRing: float = 1,
        WidthNear: float = 0.9,
        WidthHeat: float = 2,
        RadiusRing: float = 100,
        GapRing: float = 1,
        AngleCouple: float = 20,
        IsHeat: bool = False,
        IsAD: bool = True,
        Name = "Ring_Pullry",
        oplayer: LayerSpec = (1,0),
        heatlayer: LayerSpec = (2,0)
)->Component:
    c = gf.Component(Name)
    #optical part
    ring_path90 = gf.path.arc(radius=RadiusRing,angle = 90)
    ring_path_all = ring_path90+ring_path90+ring_path90+ring_path90
    ring_comp = c << gf.path.extrude(ring_path_all,width = WidthRing,layer = oplayer)
    couple_path_ring = gf.path.arc(radius=RadiusRing+GapRing+WidthNear/2+WidthRing/2,angle = AngleCouple/2)
    couple_path_euler = euler_Bend_Half(radius=RadiusRing+GapRing+WidthNear/2+WidthRing/2,angle = -AngleCouple/2)
    couple_path = couple_path_ring+couple_path_euler
    upcouple_comp1 = c << gf.path.extrude(couple_path,width = WidthNear,layer = oplayer)
    upcouple_comp1.connect("o1",destination=ring_comp.ports["o1"]).movey(2*RadiusRing+GapRing+WidthNear/2+WidthRing/2)
    upcouple_comp2 = c << gf.path.extrude(couple_path, width= WidthNear, layer=oplayer)
    upcouple_comp2.connect("o1", destination=ring_comp.ports["o1"]).movey(2*RadiusRing+GapRing+WidthNear/2+WidthRing/2)
    upcouple_comp2.rotate(center = "o1",angle=180).mirror_y(port_name="o1")

    c.add_port(name="Input", port=upcouple_comp2.ports["o2"])
    c.add_port(name="Through", port=upcouple_comp1.ports["o2"])
    c.add_port(name="RingL", port=ring_comp.ports["o1"], center=[-RadiusRing, RadiusRing], orientation=90)
    c.add_port(name="RingR", port=ring_comp.ports["o1"], center=[RadiusRing, RadiusRing], orientation=90)
    if IsAD:
        downcouple_comp1 = c << gf.path.extrude(couple_path,width = WidthNear,layer = oplayer)
        downcouple_comp1.connect("o1", destination=ring_comp.ports["o1"]).movey(-GapRing - WidthNear / 2 - WidthRing / 2)
        downcouple_comp1.mirror_y(port_name="o1")
        downcouple_comp2 = c << gf.path.extrude(couple_path, width=WidthNear, layer=oplayer)
        downcouple_comp2.connect("o1", destination=ring_comp.ports["o1"]).movey(-GapRing - WidthNear / 2 - WidthRing /2)
        downcouple_comp2.rotate(center="o1",angle=180)
        c.add_port(name="Add", port=downcouple_comp1.ports["o2"])
        c.add_port(name="Drop", port=downcouple_comp2.ports["o2"])

    #heat part
    if IsHeat:
        heat_path = gf.path.arc(radius=RadiusRing,angle=45)
        heatout_path1 = euler_Bend_Half(radius=RadiusRing/2,angle=45)
        heatout_path2 = euler_Bend_Half(radius=RadiusRing/2, angle=-45)
        heatL_comp1 = c << gf.path.extrude(heat_path+heatout_path2,width = WidthHeat,layer=heatlayer)
        heatL_comp1.connect("o1",c.ports["RingL"]).mirror_x("o1")
        heatL_comp2 = c << gf.path.extrude(heat_path+heatout_path1, width=WidthHeat, layer=heatlayer)
        heatL_comp2.connect("o1", c.ports["RingL"]).rotate(180,"o1")
        heatR_comp1 = c << gf.path.extrude(heat_path+heatout_path2, width=WidthHeat, layer=heatlayer)
        heatR_comp1.connect("o1", c.ports["RingR"])
        heatR_comp2 = c << gf.path.extrude(heat_path+heatout_path1, width=WidthHeat, layer=heatlayer)
        heatR_comp2.connect("o1", c.ports["RingR"]).mirror_y("o1")
        heatRing_route = gf.routing.get_bundle(
            [heatL_comp2.ports["o2"]],[heatR_comp2.ports["o2"]],layer = heatlayer,width = WidthHeat
        )
        for route in heatRing_route:
            c.add(route.references)
        c.add_port(name="HeatIn",port=heatL_comp1.ports["o2"])
        c.add_port(name="HeatOut", port=heatR_comp1.ports["o2"])
    return c

# %% RingPulley heat side
def RingPulley1HS(
        WidthRing: float = 1,
        WidthNear: float = 0.9,
        WidthHeat: float = 2,
        WidthRoute: float = 30,
        DeltaHeat: float = -10,
        GapRoute: float = 50,
        RadiusRing: float = 1000,
        GapRing: float = 1,
        AngleCouple: float = 20,
        IsAD: bool = True,
        Name = "Ring_Pullry",
        oplayer: LayerSpec = (1,0),
        heatlayer: LayerSpec = (2,0)
)->[Component]:
    c = gf.Component(Name)
    h = gf.Component(Name+"Heat")
    h_plus = gf.Component(Name+"Heat+")
    h_minus = gf.Component(Name+"Heat-")
    h_temp = gf.Component(Name+"HeatTemp")
    #optical part
    RingPath90 = gf.path.arc(radius=RadiusRing,angle = 90)
    RingPathAll = RingPath90+RingPath90+RingPath90+RingPath90
    RingComp = c << gf.path.extrude(RingPathAll,width = WidthRing,layer = oplayer)
    CouplePathRing = gf.path.arc(radius=RadiusRing+GapRing+WidthNear/2+WidthRing/2,angle = AngleCouple/2)
    CouplePathEuler = euler_Bend_Half(radius=RadiusRing+GapRing+WidthNear/2+WidthRing/2,angle = -AngleCouple/2)
    CouplePath = CouplePathRing+CouplePathEuler
    UpcoupleComp1 = c << gf.path.extrude(CouplePath,width = WidthNear,layer = oplayer)
    UpcoupleComp1.connect("o1",destination=RingComp.ports["o1"]).movey(2*RadiusRing+GapRing+WidthNear/2+WidthRing/2)
    UpcoupleComp2 = c << gf.path.extrude(CouplePath, width= WidthNear, layer=oplayer)
    UpcoupleComp2.connect("o1", destination=RingComp.ports["o1"]).movey(2*RadiusRing+GapRing+WidthNear/2+WidthRing/2)
    UpcoupleComp2.rotate(center = "o1",angle=180).mirror_y(port_name="o1")

    c.add_port(name="Input", port=UpcoupleComp2.ports["o2"])
    c.add_port(name="Through", port=UpcoupleComp1.ports["o2"])
    c.add_port(name="RingL", port=RingComp.ports["o1"], center=[-RadiusRing, RadiusRing], orientation=90)
    c.add_port(name="RingR", port=RingComp.ports["o1"], center=[RadiusRing, RadiusRing], orientation=90)
    if IsAD:
        downcouple_comp1 = c << gf.path.extrude(CouplePath,width = WidthNear,layer = oplayer)
        downcouple_comp1.connect("o1", destination=RingComp.ports["o1"]).movey(-GapRing - WidthNear / 2 - WidthRing / 2)
        downcouple_comp1.mirror_y(port_name="o1")
        downcouple_comp2 = c << gf.path.extrude(CouplePath, width=WidthNear, layer=oplayer)
        downcouple_comp2.connect("o1", destination=RingComp.ports["o1"]).movey(-GapRing - WidthNear / 2 - WidthRing /2)
        downcouple_comp2.rotate(center="o1",angle=180)
        c.add_port(name="Add", port=downcouple_comp1.ports["o2"])
        c.add_port(name="Drop", port=downcouple_comp2.ports["o2"])

    #heat part
    secheat1 = gf.Section(width=WidthHeat, offset=DeltaHeat, layer=heatlayer, port_names=("in", "out"))
    secout1 = gf.Section(width=RadiusRing, offset=DeltaHeat+WidthHeat/2+RadiusRing/2, layer=heatlayer, port_names=("in", "out"))
    secpad1 = gf.Section(width=RadiusRing-WidthHeat/2+DeltaHeat-GapRoute,
                         offset = -RadiusRing-(RadiusRing-WidthHeat/2+DeltaHeat-GapRoute)/2,
                         layer = heatlayer,port_names=("r_in", "r_out"))
    heatring1 = gf.CrossSection(sections=[secheat1])#heat ring
    heatring2 = gf.CrossSection(sections=[secout1])#minus route out of heat ring
    heatpad = gf.CrossSection(sections = [secpad1])#heat pad
    HP1 = h_plus << gf.path.extrude(RingPathAll, cross_section=heatring1)
    HP2 = h_minus << gf.path.extrude(RingPathAll,cross_section=heatring2)
    HR1 = h_plus << gf.c.straight(width=WidthRoute*2+2*GapRoute,length=(RadiusRing-WidthRing/2-WidthHeat+DeltaHeat-GapRoute),
                                layer = heatlayer)
    HR2 = h_minus << gf.c.straight(width=2*GapRoute,length=2*(RadiusRing),
                                layer = heatlayer)
    HPad = h_plus << gf.path.extrude(RingPathAll, cross_section=heatpad)
    HR1.connect("o1",destination=HP1.ports["in"]).rotate(-90,"o1")
    HR2.connect("o1", destination=HP1.ports["in"]).rotate(-90, "o1")
    HR2.movey(-GapRoute)
    Htotal = h << gf.geometry.boolean(A=h_plus, B=h_minus, operation = "not",layer = heatlayer)
    h.add_port(name="HeatIn", port=HP1.ports["in"], orientation=0)
    c.add_port(name="HeatIn",port=HP1.ports["in"],orientation=0)
    h.add_port(name="HeatOut", port=HP1.ports["out"], orientation=180)
    c.add_port(name="HeatOut",port=HP1.ports["out"],orientation=180)
    return [c,h]
# %% RingPulley2
@gf.cell
def RingPulley2(
        WidthRing: float = 1,
        WidthNear: float = 0.9,
        WidthHeat: float = 2,
        RadiusRing: float = 100,
        GapRing: float = 1,
        AngleCouple: float = 20,
        IsHeat: bool = False,
        Name = "Ring_Pullry2",
        oplayer: LayerSpec = (1,0),
        heatlayer: LayerSpec = (2,0)
)->Component:
    c = gf.Component(Name)
    #optical part
    ring_path90 = gf.path.arc(radius=RadiusRing,angle = 90)
    ring_path_all = ring_path90+ring_path90+ring_path90+ring_path90
    ring_comp = c << gf.path.extrude(ring_path_all,width = WidthRing,layer = oplayer)
    couple_path_ring = gf.path.arc(radius=RadiusRing+GapRing+WidthNear/2+WidthRing/2,angle = AngleCouple/2)
    couple_path_euler = euler_Bend_Half(radius=RadiusRing+GapRing+WidthNear/2+WidthRing/2,angle = (90-AngleCouple)/2,p=1)
    couple_path = couple_path_ring+couple_path_euler
    upcouple_comp1 = c << gf.path.extrude(couple_path,width = WidthNear,layer = oplayer)
    upcouple_comp1.connect("o1",destination=ring_comp.ports["o1"]).movey(2*RadiusRing+GapRing+WidthNear/2+WidthRing/2)
    upcouple_comp2 = c << gf.path.extrude(couple_path, width=WidthNear, layer=oplayer)
    upcouple_comp2.connect("o1", destination=ring_comp.ports["o1"]).movey(2*RadiusRing+GapRing+WidthNear/2+WidthRing/2)
    upcouple_comp2.rotate(center = "o1",angle=180).mirror_y(port_name="o1")

    c.add_port(name="Input", port=upcouple_comp2.ports["o2"])
    c.add_port(name="Through", port=upcouple_comp1.ports["o2"])
    c.add_port(name="RingL", port=ring_comp.ports["o1"], center=[-RadiusRing, RadiusRing], orientation=90)
    c.add_port(name="RingR", port=ring_comp.ports["o1"], center=[RadiusRing, RadiusRing], orientation=90)

    #heat part
    # if IsHeat:
    #     heat_path = gf.path.arc(radius=RadiusRing,angle=45)
    #     heatout_path1 = euler_Bend_Half(radius=RadiusRing/2,angle=45)
    #     heatout_path2 = euler_Bend_Half(radius=RadiusRing/2, angle=-45)
    #     heatL_comp1 = c << gf.path.extrude(heat_path+heatout_path2,width = WidthHeat,layer=heatlayer)
    #     heatL_comp1.connect("o1",c.ports["RingL"]).mirror_x("o1")
    #     heatL_comp2 = c << gf.path.extrude(heat_path+heatout_path1, width=WidthHeat, layer=heatlayer)
    #     heatL_comp2.connect("o1", c.ports["RingL"]).rotate(180,"o1")
    #     heatR_comp1 = c << gf.path.extrude(heat_path+heatout_path2, width=WidthHeat, layer=heatlayer)
    #     heatR_comp1.connect("o1", c.ports["RingR"])
    #     heatR_comp2 = c << gf.path.extrude(heat_path+heatout_path1, width=WidthHeat, layer=heatlayer)
    #     heatR_comp2.connect("o1", c.ports["RingR"]).mirror_y("o1")
    #     heatRing_route = gf.routing.get_bundle(
    #         [heatL_comp2.ports["o2"]],[heatR_comp2.ports["o2"]],layer = heatlayer,width = WidthHeat
    #     )
    #     for route in heatRing_route:
    #         c.add(route.references)
    #     c.add_port(name="HeatIn",port=heatL_comp1.ports["o2"])
    #     c.add_port(name="HeatOut", port=heatR_comp1.ports["o2"])
    return c
# %% RingPulley3
@gf.cell
def RingPulley3(
        WidthRing: float = 1,
        WidthNear: float = 0.9,
        WidthHeat: float = 2,
        RadiusRing: float = 100,
        GapRing: float = 1,
        AngleCouple: float = 20,
        IsHeat: bool = False,
        Name = "Ring_Pullry2",
        oplayer: LayerSpec = (1,0),
        heatlayer: LayerSpec = (2,0)
)->Component:
    '''

    Args:
        WidthRing:
        WidthNear:
        WidthHeat:
        RadiusRing:
        GapRing:
        AngleCouple:
        IsHeat:
        Name:
        oplayer:
        heatlayer:

    Returns:
        couple angle > 90 <180
    '''
    c = gf.Component(Name)
    #optical part
    ring_path90 = gf.path.arc(radius=RadiusRing,angle = 90)
    ring_path_all = ring_path90+ring_path90+ring_path90+ring_path90
    ring_comp = c << gf.path.extrude(ring_path_all,width = WidthRing,layer = oplayer)
    couple_path_ring = gf.path.arc(radius=RadiusRing+GapRing+WidthNear/2+WidthRing/2,angle = AngleCouple/2)
    couple_path_euler = euler_Bend_Half(radius=RadiusRing+GapRing+WidthNear/2+WidthRing/2,angle = (180-AngleCouple)/2,p=0.5)
    couple_path = couple_path_ring+couple_path_euler
    upcouple_comp1 = c << gf.path.extrude(couple_path,width = WidthNear,layer = oplayer)
    upcouple_comp1.connect("o1",destination=ring_comp.ports["o1"]).movey(2*RadiusRing+GapRing+WidthNear/2+WidthRing/2)
    upcouple_comp2 = c << gf.path.extrude(couple_path, width=WidthNear, layer=oplayer)
    upcouple_comp2.connect("o1", destination=ring_comp.ports["o1"]).movey(2*RadiusRing+GapRing+WidthNear/2+WidthRing/2)
    upcouple_comp2.rotate(center = "o1",angle=180).mirror_y(port_name="o1")

    c.add_port(name="Input", port=upcouple_comp2.ports["o2"])
    c.add_port(name="Through", port=upcouple_comp1.ports["o2"])
    c.add_port(name="RingL", port=ring_comp.ports["o1"], center=[-RadiusRing, RadiusRing], orientation=90)
    c.add_port(name="RingR", port=ring_comp.ports["o1"], center=[RadiusRing, RadiusRing], orientation=90)
    return c

# %% RingPulley4
@gf.cell
def RingPulley4(
        WidthRing: float = 1,
        WidthNear: float = 0.9,
        WidthHeat: float = 2,
        RadiusRing: float = 100,
        GapRing: float = 1,
        AngleCouple: float = 20,
        IsHeat: bool = False,
        Name="Ring_Pullry2",
        oplayer: LayerSpec = (1, 0),
        heatlayer: LayerSpec = (2, 0)
) -> Component:
    '''

    Args:
        WidthRing:
        WidthNear:
        WidthHeat:
        RadiusRing:
        GapRing:
        AngleCouple:
        IsHeat:
        Name:
        oplayer:
        heatlayer:

    Returns:
        couple angle > 90 <180
    '''
    c = gf.Component(Name)
    # optical part
    ring_path90 = gf.path.arc(radius=RadiusRing, angle=90)
    ring_path_all = ring_path90 + ring_path90 + ring_path90 + ring_path90
    ring_comp = c << gf.path.extrude(ring_path_all, width=WidthRing, layer=oplayer)
    couple_path_ring = gf.path.arc(radius=RadiusRing + GapRing + WidthNear / 2 + WidthRing / 2, angle=AngleCouple / 2)
    couple_path_euler_up = euler_Bend_Half(radius=RadiusRing + GapRing + WidthNear / 2 + WidthRing / 2,
                                        angle=(270 - AngleCouple) / 2, p=1)
    couple_path_euler_down = euler_Bend_Half(radius=RadiusRing + GapRing + WidthNear / 2 + WidthRing / 2,
                                        angle=(270 - AngleCouple) / 2-270, p=0.5)
    couple_path_up = couple_path_ring + couple_path_euler_up
    couple_path_down = couple_path_ring+couple_path_euler_down
    upcouple_comp1 = c << gf.path.extrude(couple_path_down, width=WidthNear, layer=oplayer)
    upcouple_comp1.connect("o1", destination=ring_comp.ports["o1"]).movey(
        2 * RadiusRing + GapRing + WidthNear / 2 + WidthRing / 2)
    upcouple_comp2 = c << gf.path.extrude(couple_path_up, width=WidthNear, layer=oplayer)
    upcouple_comp2.connect("o1", destination=ring_comp.ports["o1"]).movey(
        2 * RadiusRing + GapRing + WidthNear / 2 + WidthRing / 2)
    upcouple_comp2.rotate(center="o1", angle=180).mirror_y(port_name="o1")

    c.add_port(name="Input", port=upcouple_comp2.ports["o2"])
    c.add_port(name="Through", port=upcouple_comp1.ports["o2"])
    c.add_port(name="RingL", port=ring_comp.ports["o1"], center=[-RadiusRing, RadiusRing], orientation=90)
    c.add_port(name="RingR", port=ring_comp.ports["o1"], center=[RadiusRing, RadiusRing], orientation=90)
    return c
# %% DoubleRingPulley

def DoubleRingPulley(
        WidthRing: float = 1,
        WidthNear: float = 0.9,
        WidthHeat: float = 2,
        WidthEnd: float = 0.5,
        Pitch: float = 10,
        Period: float = 5.5,
        LengthTaper: float = 100,
        LengthR2R: float = 150,
        RadiusRing: float = 100,
        RadiusBend0: float = 40,
        DeltaRadius: float = 1,
        GapRing: float = 1,
        EndPort = [0,1,2,3],
        AngleCouple: float = 20,
        IsHeat: bool = False,
        Name = "Ring_Pullry",
        oplayer: LayerSpec = (1, 0),
        heatlayer: LayerSpec = (2, 0),
)->[Component]:
    c=gf.Component("test")
    ring1s = RingPulley(
        WidthRing=WidthRing,WidthNear=WidthNear,WidthHeat=WidthHeat,
        RadiusRing=RadiusRing,GapRing=GapRing,AngleCouple=AngleCouple,
        IsHeat=IsHeat,
        Name=Name,
        oplayer=oplayer,heatlayer=heatlayer
    )
    ring1 = c << ring1s[0]
    ring2s = RingPulley(
        WidthRing=WidthRing, WidthNear=WidthNear, WidthHeat=WidthHeat,
        RadiusRing=RadiusRing+DeltaRadius, GapRing=GapRing, AngleCouple=AngleCouple,
        IsHeat=IsHeat,
        Name=Name,
        oplayer=oplayer, heatlayer=heatlayer
    )
    ring2 = c << ring2s[0]
    str_R2R = c << gf.c.straight(width=WidthNear,length=LengthR2R,layer=oplayer)
    cir2end_elem = cir2end(
        WidthNear=WidthNear,WidthEnd=WidthEnd,LengthTaper=LengthTaper,Pitch=Pitch,RadiusBend0=RadiusBend0,Period=Period,
        oplayer=oplayer,
    )
    str_R2R.connect("o1",ring1.ports["Drop"])
    ring2.connect("Drop",str_R2R.ports["o2"]).mirror_y("Drop")
    ports_end = [ring1.ports["Add"],ring1.ports["Through"],ring2.ports["Through"],ring2.ports["Add"],ring1.ports["RingL"],
                 ring1.ports["RingR"],ring2.ports["RingL"],ring2.ports["RingR"]]
    cir_end = list(range(4))
    for portsnum in EndPort:
        cir_end[portsnum] = c << cir2end_elem
        cir_end[portsnum].connect("o1",ports_end[portsnum])
        if portsnum == 0:
            cir_end[portsnum].mirror_y("o1")
        if portsnum == 2:
            cir_end[portsnum].mirror_y("o1")
    ii = 0
    for port in ports_end:
        c.add_port(port=port,name="RingPort"+str(ii))
        ii = ii+1
    # cir2end_1 = c << cir2end_elem
    # cir2end_1.connect("o1",ring1.ports["Add"]).mirror_y("o1")
    # cir2end_2 = c << cir2end_elem
    # cir2end_2.connect("o1",ring1.ports["Through"])
    #
    # cir2end_3 = c << cir2end_elem
    # cir2end_3.connect("o1",ring2.ports["Through"]).mirror_y("o1")
    # cir2end_4 = c << cir2end_elem
    # cir2end_4.connect("o1",ring2.ports["Add"])
    #add_ports
    c.add_port(name="o1",port=ring1.ports["Input"])
    c.add_port(name="o2",port=ring2.ports["Input"])
    c.add_port(name="r2ro1",port=str_R2R.ports["o1"])
    if IsHeat:
        c.add_port(name="HeatIn1", port=ring1.ports["HeatIn"])
        c.add_port(name="HeatOut1", port=ring1.ports["HeatOut"])
        c.add_port(name="HeatIn2", port=ring2.ports["HeatIn"])
        c.add_port(name="HeatOut2", port=ring2.ports["HeatOut"])
    return [c]
# %% RunRingPulley
@gf.cell
def RunRingPulley(
        WidthRing: float = 8,
        WidthNear:float = 5,
        LengthRun: float = 200,
        RadiusRing: float = 100,
        GapRing: float = 1,
        AngleCouple:float = 20,
        IsAD:bool = True,
        oplayer: LayerSpec = "WG",
        Name:str = "RunRing_Pulley"
)->Component:
    c = gf.Component(Name)
    layer = oplayer
    secring = gf.Section(width=WidthRing, offset=0, layer=layer, port_names=("in", "out"))
    secnring = gf.Section(width=WidthNear, offset=0, layer=layer, port_names=("in", "out"))
    wgring = gf.CrossSection(sections=[secring])
    wgnear = gf.CrossSection(sections=[secnring])
    # run ring path
    rrun1 = gf.path.straight(length=LengthRun / 2)
    rring1 = gf.path.arc(radius=RadiusRing, angle=70)
    rring2 = gf.path.arc(radius=RadiusRing, angle=-70)
    rb1 = euler_Bend_Half(radius=RadiusRing, angle=20, p=0.5)
    rb2 = euler_Bend_Half(radius=RadiusRing, angle=-20, p=0.5)
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
# %% RunRingPulley2
def RunRingPulley2(
        WidthRing: float = 8,
        LengthRun: float = 200,
        RadiusRing: float = 100,
        GapRun: float = 1,
        LengthCouple:float = 200,
        IsAD:bool = True,
        IsLabels:bool = False,
        oplayer: LayerSpec = (1,0),
        heatlayer: LayerSpec = (10,0),
        Name:str = "RunRing_Pulley"
)->Component:
    c = gf.Component(Name)
    layer = oplayer
    secring = gf.Section(width=WidthRing, offset=0, layer=layer, port_names=("in", "out"))
    secnring = gf.Section(width=WidthRing, offset=0, layer=layer, port_names=("in", "out"))
    wgring = gf.CrossSection(sections=[secring])
    wgnear = gf.CrossSection(sections=[secnring])
    # run ring path
    CRunRing = gf.Component("CRunRing")
    rrun1 = gf.path.straight(length=LengthRun / 2)
    rring1 = gf.path.arc(radius=RadiusRing, angle=60)
    rring2 = gf.path.arc(radius=RadiusRing, angle=-60)
    rb1 = euler_Bend_Half(radius=RadiusRing, angle=30, p=0.5)
    rb2 = euler_Bend_Half(radius=RadiusRing, angle=-30, p=0.5)
    RingPath1 = rring1 + rb1 + rrun1
    RingPath2 = rring2 + rb2 + rrun1
    RP1 = CRunRing << gf.path.extrude(RingPath1, cross_section=wgring)
    RP2 = CRunRing << gf.path.extrude(RingPath2, cross_section=wgring)
    RP3 = CRunRing << gf.path.extrude(RingPath1, cross_section=wgring)
    RP4 = CRunRing << gf.path.extrude(RingPath2, cross_section=wgring)
    RP1.connect("out", destination=RP4.ports["out"])
    RP2.connect("in", destination=RP1.ports["in"])
    RP3.connect("out", destination=RP2.ports["out"])
    RP4.connect("in", destination=RP3.ports["in"])
    c << CRunRing
    # out port
    rcoup1 = gf.path.straight(length=LengthCouple / 2)
    rcoup2 = gf.path.straight(length=LengthCouple / 2)
    rcb1 = euler_Bend_Half(radius=RadiusRing, angle=15, p=0.5)
    rcb2 = euler_Bend_Half(radius=RadiusRing, angle=-15, p=0.5)
    RingCoup1 = rcb2+rcb1+rcoup1
    RingCoup2 = rcoup2+rcb1+rcb2
    # input through
    RC1 = c << gf.path.extrude(RingCoup1, cross_section=wgnear)
    RC2 = c << gf.path.extrude(RingCoup2, cross_section=wgnear)
    RC1.connect("out", destination=RP3.ports["out"])
    RC1.movex(-GapRun-WidthRing)
    RC2.connect("in", destination=RC1.ports["out"])
    # ports:
    c.add_port(name="Input", port=RC1.ports["in"])
    c.add_port(name="Through", port=RC2.ports["out"])
    #add drop
    if IsAD:
        RC3 = c << gf.path.extrude(RingCoup1, cross_section=wgnear)
        RC4 = c << gf.path.extrude(RingCoup2, cross_section=wgnear)
        RC3.connect("out", destination=RP1.ports["out"])
        RC3.movex(GapRun + WidthRing)
        RC4.connect("in", destination=RC3.ports["out"])
        c.add_port(name="Add", port=RC3.ports["in"])
        c.add_port(name="Drop", port=RC4.ports["out"])
    c.add_port(name="Rcen1", port=RP1.ports["out"])
    c.add_port(name="Rcen2", port=RP3.ports["out"])
    if IsLabels:
        add_labels_to_ports(c)
    return c
# %% RunRingPulley2
def RunRingPulley2HS(
        WidthRing: float = 8,
        WidthHeat: float = 8,
        WidthRoute: float = 50,
        DeltaHeat: float = -10,
        GapRoute: float = 50,
        LengthRun: float = 200,
        RadiusRing: float = 500,
        GapRun: float = 1,
        LengthCouple:float = 200,
        IsAD:bool = True,
        IsLabels:bool = False,
        IsHeat:bool = True,
        oplayer: LayerSpec = (1,0),
        heatlayer: LayerSpec = (10,0),
        Name:str = "RunRing_Pulley"
)->[Component]:
    c = gf.Component(Name)
    h = gf.Component(Name+"heat")
    layer = oplayer
    secring = gf.Section(width=WidthRing, offset=0, layer=layer, port_names=("in", "out"))
    secnring = gf.Section(width=WidthRing, offset=0, layer=layer, port_names=("in", "out"))
    wgring = gf.CrossSection(sections=[secring])
    wgnear = gf.CrossSection(sections=[secnring])
    # run ring path
    CRunRing = gf.Component("CRunRing")
    rrun1 = gf.path.straight(length=LengthRun / 2)
    rring1 = gf.path.arc(radius=RadiusRing, angle=60)
    rring2 = gf.path.arc(radius=RadiusRing, angle=-60)
    rb1 = euler_Bend_Half(radius=RadiusRing, angle=30, p=0.5)
    rb2 = euler_Bend_Half(radius=RadiusRing, angle=-30, p=0.5)
    RingPath1 = rring1 + rb1 + rrun1
    RingPath2 = rring2 + rb2 + rrun1
    RP1 = CRunRing << gf.path.extrude(RingPath1, cross_section=wgring)
    RP2 = CRunRing << gf.path.extrude(RingPath2, cross_section=wgring)
    RP3 = CRunRing << gf.path.extrude(RingPath1, cross_section=wgring)
    RP4 = CRunRing << gf.path.extrude(RingPath2, cross_section=wgring)
    RP2.connect("in", destination=RP1.ports["in"])
    RP3.connect("out", destination=RP2.ports["out"])
    RP4.connect("in", destination=RP3.ports["in"])
    c << CRunRing
    # out port
    rcoup1 = gf.path.straight(length=LengthCouple / 2)
    rcoup2 = gf.path.straight(length=LengthCouple / 2)
    rcb1 = euler_Bend_Half(radius=RadiusRing, angle=15, p=0.5)
    rcb2 = euler_Bend_Half(radius=RadiusRing, angle=-15, p=0.5)
    RingCoup1 = rcb2+rcb1+rcoup1
    RingCoup2 = rcoup2+rcb1+rcb2
    # input through
    RC1 = c << gf.path.extrude(RingCoup1, cross_section=wgnear)
    RC2 = c << gf.path.extrude(RingCoup2, cross_section=wgnear)
    RC1.connect("out", destination=RP3.ports["out"])
    RC1.movex(-GapRun-WidthRing)
    RC2.connect("in", destination=RC1.ports["out"])
    # ports:
    c.add_port(name="Input", port=RC1.ports["in"])
    c.add_port(name="Through", port=RC2.ports["out"])
    #add drop
    if IsAD:
        RC3 = c << gf.path.extrude(RingCoup1, cross_section=wgnear)
        RC4 = c << gf.path.extrude(RingCoup2, cross_section=wgnear)
        RC3.connect("out", destination=RP1.ports["out"])
        RC3.movex(GapRun + WidthRing)
        RC4.connect("in", destination=RC3.ports["out"])
        c.add_port(name="Add", port=RC3.ports["in"])
        c.add_port(name="Drop", port=RC4.ports["out"])
    c.add_port(name="Rcen1", port=RP1.ports["out"])
    c.add_port(name="Rcen2", port=RP3.ports["out"])
    if IsLabels:
        add_labels_to_ports(c)
    # heat part
    h_plus = gf.Component(Name+"Heat+")
    h_minus = gf.Component(Name + "Heat+")
    secheat1 = gf.Section(width=WidthHeat, offset=-DeltaHeat, layer=heatlayer, port_names=("in", "out"))
    secheatout1 = gf.Section(width=RadiusRing, offset=-(DeltaHeat + WidthHeat / 2 + RadiusRing / 2), layer=(30,10),port_names=("in", "out"))
    secheatpad1 = gf.Section(width=RadiusRing - (WidthHeat/2 - DeltaHeat + GapRoute),offset=RadiusRing - (RadiusRing - WidthHeat/2 + DeltaHeat - GapRoute) / 2,layer=heatlayer, port_names=("r_in", "r_out"))
    heatring1= gf.CrossSection(sections = [secheat1,secheatpad1])
    heatout1 = gf.CrossSection(sections = [secheatout1])

    secheat2 = gf.Section(width=WidthHeat, offset=DeltaHeat, layer=heatlayer, port_names=("in", "out"))
    secheatout2 = gf.Section(width=RadiusRing, offset=(DeltaHeat + WidthHeat / 2 + RadiusRing / 2), layer=(30,10),port_names=("in", "out"))
    secheatpad2 = gf.Section(width=RadiusRing - WidthHeat/2 + DeltaHeat - GapRoute,offset=-RadiusRing + (RadiusRing - WidthHeat/2 + DeltaHeat - GapRoute) / 2,layer=heatlayer, port_names=("r_in", "r_out"))
    heatring2 = gf.CrossSection(sections=[secheat2,secheatpad2])
    heatout2 = gf.CrossSection(sections=[secheatout2])
    # Heat Path
    HP1 = h_plus << gf.path.extrude(RingPath1, cross_section=heatring2)
    HP2 = h_plus << gf.path.extrude(RingPath2, cross_section=heatring1)
    HP3 = h_plus << gf.path.extrude(RingPath1, cross_section=heatring2)
    HP4 = h_plus << gf.path.extrude(RingPath2, cross_section=heatring1)
    # HP1.connect("in",destination=RP1.ports["in"]).mirror_y("in")
    HP2.connect("in", destination=HP1.ports["in"])
    HP3.connect("out", destination=HP2.ports["out"])
    HP4.connect("in", destination=HP3.ports["in"])
    # Heat
    HO1 = h_minus << gf.path.extrude(RingPath1, cross_section=heatout2)
    HO2 = h_minus << gf.path.extrude(RingPath2, cross_section=heatout1)
    HO3 = h_minus << gf.path.extrude(RingPath1, cross_section=heatout2)
    HO4 = h_minus << gf.path.extrude(RingPath2, cross_section=heatout1)
    HO2.connect("in", destination=HO1.ports["in"])
    HO3.connect("out", destination=HO2.ports["out"])
    HO4.connect("in", destination=HO3.ports["in"])
    delta = RP3.ports["in"].center-RP1.ports["in"].center
    HR1 = h_plus << gf.c.straight(width=WidthRoute * 2 + 2 * GapRoute,
                                  length=(RadiusRing - WidthRing / 2 - WidthHeat + DeltaHeat - GapRoute),
                                  layer=heatlayer)
    HR2 = h_minus << gf.c.straight(width=2 * GapRoute, length=delta[1],
                                   layer=heatlayer)
    HR1.connect("o1", destination=HP1.ports["in"]).rotate(-90, "o1")
    HR2.connect("o1", destination=HP1.ports["in"]).rotate(-90, "o1")
    HR2.movey(-GapRoute-WidthHeat+DeltaHeat)
    Htotal = h << gf.geometry.boolean(A=h_plus, B=h_minus, operation="not", layer=heatlayer)
    c.add_port(name="HeatIn",port=HP1.ports["in"],orientation=180)
    h.add_port(name="HeatIn",port=HP1.ports["in"],orientation=0)
    return [c,h]
# %% ADRAPRADR:ADRing+APRing+ADRing (with crossing ?)
@gf.cell
def ADRAPRADR(
        WidthRing: float = 1,
        WidthNear: float = 0.9,
        WidthHeat: float = 2,
        WidthSingle: float = 0.45,
        LengthTaper: float = 100,
        LengthR2R: float = 50,
        LengthR2C: float = None,
        RadiusRing: float = 100,
        DeltaRadius: float = 1,
        GapRing: float = 1,
        AngleCouple: float = 20,
        AngleCouple3: float = 40,
        IsHeat: bool = False,
        Name="Ring_Pullry",
        oplayer: LayerSpec = (602, 1),
        heatlayer: LayerSpec = (2, 0),
        CrossComp: Component = None,
)->Component:
    TriRing = gf.Component(Name)
    ring1 = TriRing << RingPulley(IsAD = True,WidthRing=WidthRing,RadiusRing=RadiusRing,oplayer = oplayer,GapRing=GapRing,
                       WidthNear=WidthNear,AngleCouple=AngleCouple,IsHeat=IsHeat,heatlayer=heatlayer,WidthHeat=WidthHeat)
    ring2 = TriRing << RingPulley(IsAD = True,WidthRing=WidthRing,RadiusRing=RadiusRing+DeltaRadius,oplayer = oplayer,GapRing=GapRing,
                       WidthNear=WidthNear,AngleCouple=AngleCouple,IsHeat=IsHeat,heatlayer=heatlayer,WidthHeat=WidthHeat)
    ring3 = TriRing << RingPulley2(WidthRing=WidthRing,RadiusRing=RadiusRing+DeltaRadius/2,oplayer = oplayer,GapRing=GapRing,
                       WidthNear=WidthNear,AngleCouple=AngleCouple3,IsHeat=IsHeat,heatlayer=heatlayer,WidthHeat=WidthHeat)
    str_r1r3 = TriRing << gf.c.straight(length = LengthR2R,width = WidthNear,layer=oplayer)
    str_r2r3 = TriRing << gf.c.straight(length=LengthR2R, width=WidthNear, layer=oplayer)
    # connect component
    str_r1r3.connect("o1",destination=ring1.ports["Drop"])
    ring3.connect("Input",destination=str_r1r3.ports["o2"]).mirror_y("Input")
    str_r2r3.connect("o1",destination=ring3.ports["Through"])
    ring2.connect("Drop",destination=str_r2r3.ports["o2"]).mirror_x("Drop")
    taper = gf.c.taper(layer=oplayer, width1=WidthNear, width2=WidthSingle, length=LengthTaper)
    taper1 = TriRing << taper
    taper2 = TriRing << taper
    taper1.connect("o1",ring1.ports["Input"])
    taper2.connect("o1",ring2.ports["Input"])
    if CrossComp != None:
        crossing = TriRing << CrossComp
        deltacross = abs(crossing.ports["o1"].center - crossing.ports["o2"].center)
        deltaring = abs(ring1.ports["Input"].center - ring2.ports["Input"].center)
        if LengthR2C == None:
            crossing.connect("o1",ring1.ports["Input"]).movex(deltaring[0]-deltacross[0])
        else:
            crossing.connect("o1",ring1.ports["Input"]).move([LengthR2C,-deltaring[0]+deltacross[0]+LengthR2C])
        route_cr1 = gf.routing.get_route(crossing.ports["o1"],taper1.ports["o2"],width = WidthSingle,layer = oplayer,radius = 50)
        route_cr2 = gf.routing.get_route(crossing.ports["o4"], taper2.ports["o2"],width = WidthSingle,layer = oplayer,radius = 50)
        TriRing.add(route_cr1.references)
        TriRing.add(route_cr2.references)
        TriRing.add_port(name="co2", port=crossing.ports["o2"])
        TriRing.add_port(name="co3", port=crossing.ports["o3"])
    #add port
    TriRing.add_port("to1",port=taper1.ports["o2"])
    TriRing.add_port("to2", port=taper2.ports["o2"])
    TriRing.add_port("r1Th",port=ring1.ports["Through"])
    TriRing.add_port("r1Ad", port=ring1.ports["Add"])
    TriRing.add_port("r2Th", port=ring2.ports["Through"])
    TriRing.add_port("r2Ad", port=ring2.ports["Add"])
    TriRing.add_port("r1L",port = ring1.ports["RingL"])
    TriRing.add_port("r1R", port=ring1.ports["RingR"])
    TriRing.add_port("r2L", port=ring2.ports["RingL"])
    TriRing.add_port("r2R", port=ring2.ports["RingR"])
    TriRing.add_port("r3L", port=ring3.ports["RingL"])
    TriRing.add_port("r3R", port=ring3.ports["RingR"])
    add_labels_to_ports(TriRing)
    return TriRing

# %% DubleRunRingPulley
@gf.cell
def DoubleRunRingPulley(
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
        ring1 = c << RunRingPulley(WidthRing=WidthRing,
                                      WidthNear=WidthNear,
                                      GapRing=GapRing,
                                      LengthRun=LengthRun,
                                      RadiusRing=RadiusRing,
                                      AngleCouple=AngleCouple,
                                      layer=layer)
        ring2 = c << RunRingPulley(WidthRing=WidthRing,
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
                              oplayer=layer)
        r2e2 = c << cir2end(WidthEnd=WidthEnd,
                              WidthNear=WidthNear,
                              LengthTaper=LengthTaper,
                              Pitch=Pitch,
                              RadiusBend0=RadiusBend0,
                              Period=Period,
                              oplayer=layer)
        r2e1.connect("o1", destination=ring1.ports["Through"])
        r2e2.connect("o1", destination=ring1.ports["Add"])
        r2e2.mirror_y(port_name="o1")
        r2e3 = c << cir2end(WidthEnd=WidthEnd,
                              WidthNear=WidthNear,
                              LengthTaper=LengthTaper,
                              Pitch=Pitch,
                              RadiusBend0=RadiusBend0,
                              Period=Period,
                              oplayer=layer)
        r2e4 = c << cir2end(WidthEnd=WidthEnd,
                              WidthNear=WidthNear,
                              LengthTaper=LengthTaper,
                              Pitch=Pitch,
                              RadiusBend0=RadiusBend0,
                              Period=Period,
                              oplayer=layer)
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

# %% Coupler2X2
@gf.cell
def Coupler2X2(
        WidthSingle:float = 0.8,
        LengthCoup:float = 100,
        LengthBridge = 300,
        Radius = 200,
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
        cbpath1 = gf.path.euler(radius=Radius,angle=30,use_eff = True)
        cbpath2 = gf.path.euler(radius=Radius,angle=-30,use_eff = True)
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
        c.add_port(name="Input1", port=CB1.ports["in"])
        c.add_port(name="Input2", port=CBs1.ports["o1"])
        c.add_port(name="Output2", port=CB2.ports["out"])
        c.add_port(name="Output1", port=CBs2.ports["o2"])
        c.add_port(name="Bridge1", port=CW1.ports["out"])
        c.add_port(name="Bridge2", port=CW2.ports["out"])
    return c
# %% SCoupler2X2
def SCoupler2X2(
        WidthSingle:float = 0.8,
        LengthCoup:float = 100,
        LengthBridge = 300,
        Radius = 200,
        GapCoup = 1,
        layer: LayerSpec = "WG",
        layers: LayerSpecs | None = None,
        Name="SCoupler2X2"
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
        cbpath1 = gf.path.euler(radius=Radius,angle=30,use_eff = True)
        cbpath2 = gf.path.euler(radius=Radius,angle=-30,use_eff = True)
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
        c.add_port(name="Input1", port=CB1.ports["in"])
        c.add_port(name="Input2", port=CBs1.ports["o1"])
        c.add_port(name="Output2", port=CB2.ports["out"])
        c.add_port(name="Output1", port=CBs2.ports["o2"])
        c.add_port(name="Bridge1", port=CW1.ports["out"])
        c.add_port(name="Bridge2", port=CW2.ports["out"])
    return c
# %% pulley 2X2Direct Coupler
def PulleyCoupler2X2(
        WidthOut:float = 1.55,
        WidthIn:float = 2,
        AngleCouple:float = 11,
        AngleIn:float = None,
        RadiusIn:float = 200,
        GapCoup:float = 0.3,
        IsParallel:bool = False,
        oplayer: LayerSpec = "WG",
        Name="Coupler2X2"
)->Component:
    '''

    Args:
        WidthOut: coupler width in nm
        WidthIn: Ring Width in nm
        AngleCouple: Couple angle in degrees
        AngleIn: RingAngle
        RadiusIn: RingRadius
        GapCoup:
        IsParallel: Is Parallel in1&2,out1&2
        oplayer: optical layer
        Name:

    Returns:component Pulley-Couple DC with port in1&2,out1&2

    '''
    c=gf.Component(Name)
    r_in = RadiusIn
    r_out =r_in+WidthOut/2+WidthIn/2+GapCoup
    sout = gf.Section(width=WidthOut, offset=0, layer=oplayer, port_names=("in", "out"))
    sin = gf.Section(width=WidthIn, offset=0, layer=oplayer, port_names=("in", "out"))
    csout = gf.CrossSection(sections=[sout])
    csin = gf.CrossSection(sections=[sin])
    po_ring = gf.path.arc(radius=r_out,angle = AngleCouple/2)
    po_euler = euler_Bend_Half(radius=r_out,angle=-AngleCouple/2, p=1)
    po = po_euler+po_ring
    if AngleIn == None:
        AngleIn = AngleCouple*2
    pi_euler = euler_Bend_Half(radius=r_in,angle=-AngleIn/2, p=1)
    if IsParallel:
        pi_ring = gf.path.arc(radius=r_in, angle=AngleIn/2)
        pi = pi_euler+pi_ring
    else:
        pi_ring = gf.path.arc(radius=r_in, angle=-AngleIn/2)
        pi = pi_ring
    co = gf.path.extrude(po,cross_section=csout)
    ci = gf.path.extrude(pi,cross_section=csin)
    co1 = c << co
    co2 = c << co
    ci1 = c << ci
    ci2 = c << ci
    co2.connect("in",destination=co1.ports["in"])
    ci1.connect("in",destination=co1.ports["in"]).mirror_x("in")
    ci1.movey(WidthOut/2+WidthIn/2+GapCoup)
    ci2.connect("in", destination=ci1.ports["in"])
    co1.mirror_y("in")
    c.add_port("in1",port=ci1.ports["out"])
    c.add_port("in2", port=co1.ports["out"])
    c.add_port("out1", port=ci2.ports["out"])
    c.add_port("out2", port=co2.ports["out"])
    add_labels_to_ports(c)
    return c
# %% SagnacRing
def SagnacRing(
        WidthOut:float = 1.55,
        WidthIn:float = 2,
        LengthTaper:float = 200,
        AngleCouple = 11,
        RadiusIn = 200,
        RadiusBend = 100,
        GapCoup = 0.3,
        oplayer: LayerSpec = "WG",
        Name="SagnacRing"
)->Component:

    c = Component(Name)
    PC = c << PulleyCoupler2X2(WidthIn=WidthIn,WidthOut=WidthOut,AngleCouple=AngleCouple,RadiusIn=RadiusIn,
                          GapCoup=GapCoup,oplayer=oplayer,IsParallel=False,AngleIn=90)
    taper_coup2ring = c << gf.c.taper(width1 = WidthOut,width2 = WidthIn,length = LengthTaper,layer=oplayer)
    taper_coup2ring.connect("o1",destination=PC.ports["out2"])
    bendpath_ring2coup = euler_Bend_Half(angle=-45,radius = RadiusBend,p=1,use_eff=False)
    bend_ring2coup = c << gf.path.extrude(bendpath_ring2coup,width=WidthOut,layer=oplayer)
    bend_ring2coup.connect("o1",destination=PC.ports["out1"])
    bendpath_ring2out = euler_Bend_Half(angle=45,radius = RadiusBend,p=1,use_eff=False)
    bend_ring2out = c << gf.path.extrude(bendpath_ring2out,width=WidthOut,layer=oplayer)
    bend_ring2out.connect("o1",destination=PC.ports["in1"])
    routering = gf.routing.get_bundle([bend_ring2coup.ports["o2"]],[taper_coup2ring.ports["o2"]],width = WidthIn,layer=oplayer
                                     ,bend=gf.c.bend_euler(width = WidthIn,radius = RadiusBend,with_arc_floorplan=False,p=0.8),
                                    )
    for route in routering:
        c.add(route.references)
    bend = c << gf.c.bend_euler(angle=90,width=WidthOut,layer=oplayer,radius=RadiusBend,with_arc_floorplan=False,p=1)
    bend.connect("o1",bend_ring2out.ports["o2"])
    c.add_port("input",port=PC.ports["in2"])
    c.add_port("output",port=bend.ports["o2"])
    return c

# %% OpenPad
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
# %% DBR
@gf.cell
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
# %% DBRFromCsv
@gf.cell
def DBRFromCsv(
        Width1:float = 2,
        Width2:float = 1,
        WidthHeat:float = 4,
        WidthRoute:float = 10,
        CSVName = "D:/Mask Download/Temp202311_LN_ZJ/单_D01.5e-25_k0.5_1500-1600.csv",
        IsHeat:bool = False,
        hetlayer:LayerSpec = (31,0),
        oplayer: LayerSpec = (1,0),
        Name = "DBR"
)->Component:
    c=gf.Component(Name)
    lengthrows = csv.reader(open(CSVName))
    Period = len(list(lengthrows))
    lengthrows = csv.reader(open(CSVName))
    r1 = []
    r2 = []
    for i,length in enumerate(lengthrows):
        length0 = float(length[0])
        length1 = float(length[1])
        if length0 < 1e-3:
            length0 = 1e6*length0
            length1 = 1e6*length1
        if i == 0:
            r1.append(c << gf.c.straight(length=length0, width=Width1, layer=oplayer))
            r2.append(c << gf.c.straight(length=length1, width=Width2, layer=oplayer))
            r2[0].connect(port="o1", destination=r1[0].ports["o2"])
        else:
            r1.append(c << gf.c.straight(length=length0, width=Width1, layer=oplayer))
            r2.append(c << gf.c.straight(length=length1, width=Width2, layer=oplayer))

            r1[i].connect(port="o1", destination=r2[i-1].ports["o2"])
            r2[i].connect(port="o1", destination=r1[i].ports["o2"])
    c.add_port("o1",port=r1[0].ports["o1"])
    c.add_port("o2",port=r2[-1].ports["o2"])

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
# %% OffsetRamp
@gf.cell
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


# %% GSGELE
@gf.cell
def GSGELE(
        WidthG:float = 80,
        WidthS:float = 25,
        GapGS:float = 6,
        LengthEle:float = 10000,
        IsPad:bool = False,
        Is2Pad:bool =False,
        PitchPad:float = 100,
        WidthOpen:float = 40,
        Enclosure:float = 10,
        openlayer:LayerSpec = (3,5),
        elelayer:LayerSpec = (2,4),
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
    GSGPadarray = gf.Component()
    GSGPad1 = GSGPadarray << OpenPad(WidthOpen=WidthOpen, Enclosure=Enclosure, elelayer=elelayer,
                                     openlayer=openlayer)
    GSGPad2 = GSGPadarray << OpenPad(WidthOpen=WidthOpen, Enclosure=Enclosure, elelayer=elelayer,
                                     openlayer=openlayer)
    GSGPad3 = GSGPadarray << OpenPad(WidthOpen=WidthOpen, Enclosure=Enclosure, elelayer=elelayer,
                                     openlayer=openlayer)
    GSGPad2.movey(PitchPad)
    GSGPad3.movey(PitchPad * 2)
    GSGPadarray.add_port("Pr1", port=GSGPad1.ports["right"])
    GSGPadarray.add_port("Pl1", port=GSGPad1.ports["left"])
    GSGPadarray.add_port("Pr2", port=GSGPad2.ports["right"])
    GSGPadarray.add_port("Pl2", port=GSGPad2.ports["left"])
    GSGPadarray.add_port("Pr3", port=GSGPad3.ports["right"])
    GSGPadarray.add_port("Pl3", port=GSGPad3.ports["left"])
    if Is2Pad:
        IsPad=True
        GPa2 = c << GSGPadarray
        GPa2.connect("Pl1", destination=c.ports["Gout1"]).move([-PitchPad / 3, PitchPad / 5])
        # Gin1
        pos_diff = GPa2.ports["Pl1"].center - c.ports["Gout1"].center
        G2Pa21 = c << OffsetRamp(length=PitchPad / 3, width1=WidthOpen + 2 * Enclosure, width2=80, offset=-pos_diff[1],
                                layer=elelayer)
        G2Pa21.connect("o2", destination=c.ports["Gout1"])
        # Sin
        pos_diff = GPa2.ports["Pl2"].center - c.ports["Sout"].center
        G2Pa22 = c << OffsetRamp(length=PitchPad / 3, width1=WidthOpen + 2 * Enclosure, width2=25, offset=-pos_diff[1],
                                layer=elelayer)
        G2Pa22.connect("o2", destination=c.ports["Sout"])
        # Gin2
        pos_diff = GPa2.ports["Pl3"].center - c.ports["Gout2"].center
        G2Pa23 = c << OffsetRamp(length=PitchPad / 3, width1=WidthOpen + 2 * Enclosure, width2=80, offset=-pos_diff[1],
                                layer=elelayer)
        G2Pa23.connect("o2", destination=c.ports["Gout2"])
    if IsPad:
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
# %% ViaArray
@gf.cell
def ViaArray(
        WidthVia: float = 0.2,
        Spacing: float = 1,
        Col: float = 5,
        Row: float = 10,
        Enclosure: float = 1,
        IsEn:bool = False,
        ViaLayer:LayerSpec = (200,1),
        ViaEnLayers:LayerSpec = [(200,10)],
)->Component:
    via_array = gf.Component("ViaArray")
    via = gf.c.straight(width=WidthVia, length=WidthVia, layer=ViaLayer)
    via_ref = [[j for j in range(Col)] for i in range(Row)]
    for i in range(Row):
        for j in range(Col):
            via_ref[i][j] = via_array << via
            via_ref[i][j].move([(i)*Spacing,(j-Col/2+1/2)*Spacing])
            via_ref[i][j].rotate(90)
    if IsEn:
        WidthEn = Spacing*(Row-1)+WidthVia+2*Enclosure
        LengthEn = Spacing*(Col-1)+WidthVia+2*Enclosure
        for layer in ViaEnLayers:
            encl = via_array << gf.c.straight(width=LengthEn, length=WidthEn, layer=layer)
            encl.move([-Enclosure,0])
            encl.rotate(90)
        for ref in via_array:
                ref.move([0,Enclosure])
    via_array.add_port("center", port=via_ref[0][0].ports["o1"],
                       center=[0, Spacing * (Row - 1) / 2 + WidthVia / 2 + Enclosure * IsEn])
    via_array.add_port("up", port=via_ref[0][0].ports["o1"],
                       center=[0, Spacing * (Row - 1) + WidthVia+ 2* Enclosure * IsEn], orientation=90)
    via_array.add_port("down", port=via_ref[0][0].ports["o1"],
                       center=[0, 0],
                       orientation=270)
    via_array.add_port("right", port=via_ref[0][0].ports["o1"],
                       center=[(Spacing * (Col - 1) / 2 + WidthVia / 2 + Enclosure * IsEn),
                               Spacing * (Row - 1) / 2 + WidthVia / 2 + Enclosure * IsEn],
                       orientation=0)
    via_array.add_port("left", port=via_ref[0][0].ports["o1"],
                       center=[-(Spacing * (Col - 1) / 2 + WidthVia / 2 + Enclosure * IsEn),
                               Spacing * (Row - 1) / 2 + WidthVia / 2 + Enclosure * IsEn],
                       orientation=180)
    return via_array
# %% RingHeater
@gf.cell
def RingHeater(
        WidthRing: float = 1,
        WidthHeat: float = 2,
        WidthCLD: float = 3,
        WidthRoute: float = 10,
        LengthRoute: float = 30,
        RadiusRing: float = 100,
        GapHeat: float = 1.5,
        NumOfVia: int = 12,
        Name = "RingHeater",
        oplayer: LayerSpec = (602,1),
        heaterlayer: LayerSpec = (614,1),
        hwglayer: LayerSpec = (600,1),
        hslablayer: LayerSpec = (600,10),
        routelayer: LayerSpec = (641,1),
        IsSlab: bool = True,
        IsVia:bool = True,
        IsRoute: bool = True,
        ViaComp:Component = ViaArray(),
) -> Component:
    """Returns a ring heater.

    Args:
        WidthRing:
        WidthHeat:
        WidthCLD:Cladding enclusure
        RadiusRing:
        GapHeat:
        oplayer:ring layer
        heaterlayer:heater layer
        hwglayer: heater waveguide layer(in SOI fab)
        hslab: heater slab layer(in SOI fab)
        IsSlab: bool = False:if is slab waveguide layer
    """
    c = gf.Component(Name)
    deltaVCY = ViaComp.ports["up"].center[1]-ViaComp.ports["down"].center[1]
    deltaVCX = ViaComp.ports["right"].center[0] - ViaComp.ports["left"].center[0]
    deltaR = WidthHeat+GapHeat+WidthRing/2+deltaVCY
    # section
    S_heater = gf.Section(width = WidthHeat, offset = -GapHeat-WidthRing/2-WidthHeat/2,layer = heaterlayer,port_names = ("hin","hout"))
    S_hwg = gf.Section(width = 2*WidthHeat+2*GapHeat+WidthRing,offset = 0,layer = hwglayer)
    S_slab = gf.Section(width=2 * WidthHeat + 2 * GapHeat + WidthRing, offset=0, layer=hslablayer)
    offsetVC = gf.Component()
    VC = offsetVC << ViaComp
    VC.movey(GapHeat+WidthRing/2+WidthHeat/2+0.5)
    CAP_via = gf.cross_section.ComponentAlongPath(component=offsetVC,
                                                  spacing=2 * np.pi * (RadiusRing) / (NumOfVia),
                                                  port_names=("o1"),padding = -10)
    #CAP_R1,2 need component
    CAP_Rin_comp = gf.Component()
    CAP_R0 = CAP_Rin_comp << gf.c.straight(width=deltaVCX,length=LengthRoute,layer=routelayer,add_pins=False)
    CAP_R0.rotate(90).movey(deltaR-deltaVCY-0.5)
    CAP_Rout_comp = gf.Component()
    CAP_R1 = CAP_Rout_comp << gf.c.straight(width=deltaVCX,length=LengthRoute,layer=routelayer,add_pins=False)
    CAP_R1.rotate(90).movey(-LengthRoute+deltaR)
    CAP_Rin = gf.cross_section.ComponentAlongPath(component=CAP_Rout_comp,
                                                  spacing=4 * np.pi * (RadiusRing) / (NumOfVia),padding = -50
                                                  )
    CAP_Rout = gf.cross_section.ComponentAlongPath(component=CAP_Rin_comp,
                                                  spacing=4 * np.pi * (RadiusRing) / (NumOfVia),
                                                  padding=-4 * np.pi * (RadiusRing) / (NumOfVia)*2)
    # Metal in out
    S_RinRing = gf.Section(width = WidthRoute,offset = -deltaR-LengthRoute+deltaVCY,layer = routelayer)
    S_RoutRing = gf.Section(width = WidthRoute,offset = -deltaR+LengthRoute,layer = routelayer)
    # Cross-Section
    CS_hs = gf.CrossSection(sections = [S_heater,S_hwg])
    CS_via = gf.CrossSection(components_along_path=[CAP_via])
    CS_Rin = gf.CrossSection(components_along_path=[CAP_Rin])
    CS_Rout = gf.CrossSection(components_along_path=[CAP_Rout])
    CS_RRing = gf.CrossSection(sections = [S_RinRing,S_RoutRing])
    # optical part
    ring_path90 = gf.path.arc(radius=RadiusRing,angle = 90)
    ring_path_all = ring_path90+ring_path90+ring_path90+ring_path90
    ring_comp = c << gf.path.extrude(ring_path_all,width = WidthRing,layer = oplayer)
    heater = c << gf.path.extrude(ring_path_all,cross_section=CS_hs)

    # add port
    c.add_port(name="RingL", port=ring_comp.ports["o1"], center=[-RadiusRing, RadiusRing], orientation=90)
    c.add_port(name="RingR", port=ring_comp.ports["o1"], center=[RadiusRing, RadiusRing], orientation=90)

    # VIA
    if IsVia:
        vias = c << gf.path.extrude(ring_path_all,cross_section=CS_via)
        # vias_wg = c << gf.geometry.offset(vias,distance=0.5,layer=hwglayer)
        # vias_heater = c << gf.geometry.offset(vias,distance=0.5,layer=heaterlayer)
    # heater waveguide and slab
    if IsSlab:
        gds_hslab_pre = gf.geometry.offset(c, distance=WidthCLD+0.8)
        gds_hslab = c << gf.geometry.offset(gds_hslab_pre, distance=-0.8,layer=hslablayer)
    if IsRoute:
        routes1 = c << gf.path.extrude(ring_path_all,cross_section=CS_Rin)
        routes2 = c << gf.path.extrude(ring_path_all, cross_section=CS_Rout)
        routering = c << gf.path.extrude(ring_path_all,cross_section=CS_RRing)
    add_labels_to_ports(c)
    return c
# %% QRcode of team website
@gf.cell
def TWQRcode(
    Size:float = 10,
    lglayer:LayerSpec = (10,0)
)->Component:
    logo = gf.Component("logo")
    qrcode = logo << gf.c.qrcode(data = 'https://photonics.pku.edu.cn/',psize = Size,layer = lglayer)
    return logo
# %% Component shift
def shift_component(component: Component, dx: float, dy: float) -> Component:
    """Shifts all elements in the component by (dx, dy).

    Args:
        component: The component to shift.
        dx: The shift in the x-direction.
        dy: The shift in the y-direction.

    Returns:
        A new component with all elements shifted.
    """
    new_component = Component()  # 创建一个新的组件实例

    # 将原始组件的所有元素平移并添加到新组件中
    for ref in component.references:
        new_component.add_ref(ref.parent).move([dx, dy])

    for polygon in component.polygons:
        new_component.add_polygon(polygon.points + (dx, dy), layer=polygon.layer)

    for label in component.labels:
        new_component.add_label(
            text=label.text,
            position=(label.origin[0] + dx, label.origin[1] + dy),
            layer=label.layer,
            magnification=label.magnification,
            rotation=label.rotation,
        )

    # 平移所有端口
    for port in component.ports.values():
        new_component.add_port(port=port.copy().move([dx, dy]))

    return new_component
# %% TotalComponent
r_euler_false = 500
r_euler_true =500*1.5
# % SingleRing: simple straight pulley
@gf.cell
def SingleRing(
        r_ring: float = 120,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float = 5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 150,
        length_total:float = 10000,
        pos_ring:float = 500,
        delta_io = 500,
        gap_rc: float = 1,
        tin:Component = None,
        tout:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("RingAll")
    ring = gf.Component("Ring")
    ring0 = ring << RingPulley(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,IsAD = False,
        AngleCouple=angle_rc,WidthHeat=width_heat
    )
    taper_s2n1 = ring << gf.c.taper(width1 = width_single,width2 = width_near,length=length_taper,layer=oplayer)
    taper_s2n2 = ring << gf.c.taper(width1 = width_near,width2 = width_single,length=length_taper,layer=oplayer)
    taper_s2n1.connect("o2",ring0.ports["Input"])
    bend_thr1 = ring << gf.c.bend_euler(width = width_near,radius=r_ring*2/3,layer=oplayer,angle = 90)
    bend_thr1.connect("o1",ring0.ports["Through"])
    taper_s2n2.connect("o1", bend_thr1.ports["o2"]).mirror_x("o1")
    ring.add_port("Input",port=ring0.ports["Input"])
    ring.add_port("RingL", port=ring0.ports["RingL"])
    ring.add_port("RingR", port=ring0.ports["RingR"])
    ring.add_port("Ts2n1_o1",port=taper_s2n1.ports["o1"])
    ring.add_port("Ts2n2_o2", port=taper_s2n2.ports["o2"])
    Ring = sr << ring
    if tin != None:
        tdring = sr << tin
        Ring.connect("Input", destination=tdring.ports["o2"]).movex(pos_ring).mirror_y("Input")
        delta = Ring.ports["Ts2n1_o1"].center-tdring.ports["o2"].center
        str_td2r = sr << gf.c.straight(length=delta[0],layer = oplayer,width = width_single)
        str_td2r.connect("o1",tdring.ports["o2"])
        sr.add_port("input", port=tdring.ports["o1"])
    if tout != None:
        tetring = sr << tout
        delta = tetring.ports["o2"].center - tetring.ports["o1"].center
        tetring.connect("o1", tdring.ports["o1"]).mirror_x("o1").movex(length_total - delta[0] + 75)
        tetring.movey(-delta_io)
        str_tet2r = gf.routing.get_bundle(tetring.ports["o1"],Ring.ports["Ts2n2_o2"],layer = oplayer,width = width_single
                                                    ,radius = r_euler_true)
        for route in str_tet2r:
            sr.add(route.references)
        sr.add_port("output", port=tetring.ports["o2"])


    sr.add_port("RingC",port=tetring.ports["o1"],center=Ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    return sr
# %% SingleRing1_2: add-drop ring
@gf.cell
def SingleRing1_2(
        r_ring: float = 120,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float =5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 150,
        length_total:float = 2000,
        pos_ring:float = 500,
        gap_rc: float = 1,
        tet:Component = None,
        td:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("Ring")
    # Section CrossSection
    S_near = gf.Section(width=width_near,layer=oplayer,port_names=("o1", "o2"))
    CS_near = gf.CrossSection(sections=[S_near])
    # component
    tdring = sr << td
    tetring_th = sr << tet
    tetring_ad = sr << tet
    tetring_dr = sr << tet
    taper_s2n = gf.c.taper(width1 = width_single,width2=width_near,length=length_taper,layer=oplayer)
    taper_s2n_in = sr << taper_s2n
    taper_s2n_th = sr << taper_s2n
    taper_s2n_ad = sr << taper_s2n
    taper_s2n_dr = sr << taper_s2n
    ring = sr << RingPulley(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        AngleCouple=angle_rc,WidthHeat=width_heat,IsAD=True
    )
    taper_s2n_in.movex(pos_ring-length_taper)
    ring.connect("Input",destination=taper_s2n_in.ports["o2"]).mirror_y("Input")
    length_tet = abs(tetring_th.ports["o1"].center-tetring_th.ports["o2"].center)
    # add
    taper_s2n_ad.connect("o2",destination=ring.ports["Add"])
    tetring_ad.connect("o1",destination=taper_s2n_ad.ports["o1"])
    tetring_ad.movex(length_total-length_tet[0]-taper_s2n_ad.ports["o1"].center[0])
    # through
    bend_th1 = sr << gf.c.bend_euler(width=width_near,layer=oplayer,angle=-90,
                                     radius=r_ring / 2 + r_euler_false / 2+r_ring*np.sin((20-angle_rc/2)*3.14/180))
    bend_th2 = sr << gf.c.bend_euler(width=width_near, layer=oplayer, angle=90,
                                     radius=r_ring / 2 + r_euler_false / 2+r_ring*np.sin((10-angle_rc/2)*3.14/180))
    bend_th1.connect("o1",destination=ring.ports["Through"])
    delta = bend_th1.ports["o2"].center-taper_s2n_ad.ports["o2"].center
    bend_th2.connect("o1",destination=bend_th1.ports["o2"]).movey(-delta[1]+r_ring / 2 + r_euler_false / 2+15++r_ring*np.sin((10-angle_rc/2)*3.14/180))
    route = gf.routing.get_route(bend_th2.ports["o1"],bend_th1.ports["o2"],width=width_near,layer=oplayer)
    sr.add(route.references)
    taper_s2n_th.connect("o2",destination=bend_th2.ports["o2"])
    tetring_th.connect("o1",destination=taper_s2n_th.ports["o1"])
    tetring_th.movex(length_total-length_tet[0]-taper_s2n_th.ports["o1"].center[0])
    # drop
    taper_s2n_dr.connect("o2",destination=ring.ports["Drop"]).mirror_x("o2")
    taper_s2n_dr.movey(-35-r_ring*(1-np.cos(angle_rc/2*3.14/180)))
    bend_dr1 = sr << gf.c.bend_euler(width=width_near,layer=oplayer,angle=-30,radius=r_euler_false*3)
    bend_dr1.connect("o1", destination=ring.ports["Drop"])
    bend_dr2 = sr << gf.c.bend_euler(width=width_near,layer=oplayer,angle=210,radius=r_euler_false*2/3)
    bend_dr2.connect("o1",destination=bend_dr1.ports["o2"])
    route = gf.routing.get_route_sbend(bend_dr2.ports["o2"],taper_s2n_dr.ports["o2"],cross_section=CS_near)
    sr.add(route.references)
    tetring_dr.connect("o1",destination=taper_s2n_dr.ports["o1"])
    tetring_dr.movex(length_total-length_tet[0]-taper_s2n_dr.ports["o1"].center[0])
    # route io
    route_io = gf.routing.get_bundle(
        [tdring.ports["o2"],taper_s2n_ad.ports["o1"],taper_s2n_dr.ports["o1"],taper_s2n_th.ports["o1"]],
        [taper_s2n_in.ports["o1"],tetring_ad.ports["o1"],tetring_dr.ports["o1"],tetring_th.ports["o1"]],
        width=width_single,layer = oplayer
    )
    for route in route_io:
        sr.add(route.references)
    # for route in str_tet2r:
    #     sr.add(route.references)
    # # sr_cld1 = gf.geometry.offset(sr, distance=width_cld, layer=LAYER.CLD2)
    # # sr_cld2 = gf.geometry.offset(sr_cld1, distance=-0.8, layer=LAYER.CLD2)
    # # sr.add_ref(sr_cld2)
    sr.add_port("input",port=tdring.ports["o1"])
    sr.add_port("through",port=tetring_th.ports["o2"])
    sr.add_port("drop", port=tetring_dr.ports["o2"])
    sr.add_port("add", port=tetring_ad.ports["o2"])
    sr.add_port("RingC",port=sr.ports["input"],center=ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    add_labels_to_ports(sr)
    return sr
# %% SingleRing: simple straight pulley
@gf.cell
def SingleRing1_3(
        r_ring: float = 120,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float = 5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 150,
        length_total:float = 10000,
        pos_ring:float = 500,
        gap_rc: float = 1,
        tin:Component = None,
        tout:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("RingAll")
    ring = gf.Component("Ring")
    Cring = RingPulley(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        AngleCouple=angle_rc,WidthHeat=width_heat,IsAD = False,
    )
    ring0 = ring << Cring
    taper_s2n1 = ring << gf.c.taper(width1 = width_single,width2 = width_near,length=length_taper,layer=oplayer)
    taper_s2n2 = ring << gf.c.taper(width1 = width_near,width2 = width_single,length=length_taper,layer=oplayer)
    taper_s2n1.connect("o2",ring0.ports["Input"])
    taper_s2n2.connect("o1", ring0.ports["Through"])
    ring.add_port("Input",port=ring0.ports["Input"])
    ring.add_port("RingL", port=ring0.ports["RingL"])
    ring.add_port("RingR", port=ring0.ports["RingR"])
    ring.add_port("Ts2n1_o1",port=taper_s2n1.ports["o1"])
    ring.add_port("Ts2n2_o2", port=taper_s2n2.ports["o2"])
    Ring = sr << ring
    if tin != None:
        tdring = sr << tin
        Ring.connect("Input", destination=tdring.ports["o2"]).movex(pos_ring).mirror_y("Input")
        delta = Ring.ports["Ts2n1_o1"].center-tdring.ports["o2"].center
        str_td2r = sr << gf.c.straight(length=delta[0],layer = oplayer,width = width_single)
        str_td2r.connect("o1",tdring.ports["o2"])
        sr.add_port("input", port=tdring.ports["o1"])
    if tout != None:
        tetring = sr << tout
        delta = tetring.ports["o2"].center - tetring.ports["o1"].center
        tetring.connect("o1", tdring.ports["o1"]).mirror_x("o1").movex(length_total - delta[0] + 75)
        str_tet2r = gf.routing.get_bundle(tetring.ports["o1"],Ring.ports["Ts2n2_o2"],layer = oplayer,width = width_single
                                                    ,radius = r_euler_true)
        for route in str_tet2r:
            sr.add(route.references)
        sr.add_port("output", port=tetring.ports["o2"])
    sr.add_port("RingC",port=tetring.ports["o1"],center=Ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    return sr
def SingleRing1_3hs(
        r_ring: float = 120,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float = 5,
        width_single:float =1,
        delta_heat:float = 10,
        angle_rc: float = 20,
        gap_route: float = 50,
        length_taper: float = 150,
        length_total:float = 10000,
        pos_ring:float = 500,
        gap_rc: float = 1,
        tin:Component = None,
        tout:Component = None,
        oplayer: LayerSpec = LAYER.WG,
        heatlayer: LayerSpec = LAYER.M1,
) -> [Component]:
    '''
    hs: heat side
    '''
    sr = gf.Component("RingAll")
    sh = gf.Component("Heat")
    ring = gf.Component("Ring")
    Cring = RingPulley1HS(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,RadiusRing=r_ring,
        AngleCouple=angle_rc,WidthHeat=width_heat,IsAD = False,GapRoute=gap_route,
        oplayer=oplayer,heatlayer=heatlayer,DeltaHeat = delta_heat
    )
    heat0 = sh << Cring[1]
    ring0 = ring << Cring[0]
    taper_s2n1 = ring << gf.c.taper(width1 = width_single,width2 = width_near,length=length_taper,layer=oplayer)
    taper_s2n2 = ring << gf.c.taper(width1 = width_near,width2 = width_single,length=length_taper,layer=oplayer)
    taper_s2n1.connect("o2",ring0.ports["Input"])
    taper_s2n2.connect("o1", ring0.ports["Through"])
    ring.add_port("Input",port=ring0.ports["Input"])
    ring.add_port("RingL", port=ring0.ports["RingL"])
    ring.add_port("RingR", port=ring0.ports["RingR"])
    ring.add_port("HeatIn",port=ring0.ports["HeatIn"])
    ring.add_port("Ts2n1_o1",port=taper_s2n1.ports["o1"])
    ring.add_port("Ts2n2_o2", port=taper_s2n2.ports["o2"])
    Ring = sr << ring

    if tin != None:
        tdring = sr << tin
        Ring.connect("Input", destination=tdring.ports["o2"]).movex(pos_ring).mirror_y("Input")
        delta = Ring.ports["Ts2n1_o1"].center-tdring.ports["o2"].center
        str_td2r = sr << gf.c.straight(length=delta[0],layer = oplayer,width = width_single)
        str_td2r.connect("o1",tdring.ports["o2"])
        sr.add_port("input", port=tdring.ports["o1"])
    if tout != None:
        tetring = sr << tout
        delta = tetring.ports["o2"].center - tetring.ports["o1"].center
        tetring.connect("o1", tdring.ports["o1"]).mirror_x("o1").movex(length_total - delta[0] + 75)
        str_tet2r = gf.routing.get_bundle(tetring.ports["o1"],Ring.ports["Ts2n2_o2"],layer = oplayer,width = width_single
                                                    ,radius = r_euler_true)
        for route in str_tet2r:
            sr.add(route.references)
        sr.add_port("output", port=tetring.ports["o2"])
    sr.add_port("RingC",port=Ring.ports["Input"],center=Ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    sh.add_port("RingC",port=sr.ports["RingC"],orientation=180)
    sr.add_port("HeatIn",port=Ring.ports["HeatIn"])
    sh.add_port("HeatIn",port=heat0.ports["HeatIn"])
    add_labels_to_ports(sr)
    add_labels_to_ports(sh,label_layer=(30,10))

    return [sr,sh]
# %% SingleRing2: bend PulleyRing
@gf.cell
def SingleRing2(
        r_ring: float = 120,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float =5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 150,
        length_total:float = 10000,
        pos_ring:float = 500,
        delta_io = 500,
        gap_rc: float = 1,
        tet:Component = None,
        td:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("Ring")
    ring = sr << RingPulley2(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        AngleCouple=angle_rc,WidthHeat=width_heat
    )
    if td != None:
        tdring = sr << td
        ring.connect("Input", destination=tdring.ports["o2"]).movex(pos_ring).mirror_y("Input")
    if tet != None:
        tetring = sr << tet
        delta = tetring.ports["o2"].center - tetring.ports["o1"].center
        tetring.connect("o1", tdring.ports["o1"]).mirror_x("o1").movex(length_total - delta[0] + 75)
        tetring.movey(-delta_io)
    taper_s2n1 = sr << gf.c.taper(width1 = width_single,width2 = width_near,length=length_taper,layer=oplayer)
    taper_s2n2 = sr << gf.c.taper(width1 = width_near,width2 = width_single,length=length_taper,layer=oplayer)
    taper_s2n1.connect("o2",ring.ports["Input"])
    bend_thr1 = sr << gf.c.bend_euler(width = width_near,radius=r_ring*2/3,layer=oplayer,angle = -90)
    bend_thr1.connect("o1",ring.ports["Through"])

    taper_s2n2.connect("o1", bend_thr1.ports["o2"]).mirror_x("o1")
    if td != None:
        delta = taper_s2n1.ports["o1"].center-tdring.ports["o2"].center
        str_td2r = sr << gf.c.straight(length=delta[0],layer = oplayer,width = width_single)
        str_td2r.connect("o1",tdring.ports["o2"])
        sr.add_port("input", port=tdring.ports["o1"])
    if tet != None:
        str_tet2r = gf.routing.get_bundle(tetring.ports["o1"],taper_s2n2.ports["o2"],layer = oplayer,width = width_single
                                                    ,radius = r_euler_true)
        for route in str_tet2r:
            sr.add(route.references)
        sr.add_port("output", port=tetring.ports["o2"])
    sr.add_port("RingC",port=tetring.ports["o1"],center=ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    return sr

# %% SingleRing2_2: pulleyRing taper_s2n after output bend
@gf.cell
def SingleRing2_2(
        r_ring: float = 120,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float =5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 150,
        length_total:float = 10000,
        pos_ring:float = 500,
        gap_rc: float = 1,
        tin:Component = None,
        tout:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("Ring")
    ring = sr << RingPulley2(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        AngleCouple=angle_rc,WidthHeat=width_heat
    )
    tdring = sr << tin
    tetring = sr << tout
    ring.connect("Input",destination=tdring.ports["o2"]).movex(pos_ring).mirror_y("Input")
    delta = tetring.ports["o2"].center-tetring.ports["o1"].center
    tetring.connect("o1",tdring.ports["o1"]).mirror_x("o1").movex(length_total-delta[0])
    taper_s2n1 = sr << gf.c.taper(width1 = width_single,width2 = width_near,length=length_taper,layer=oplayer)
    taper_s2n1.connect("o2",ring.ports["Input"])
    delta = taper_s2n1.ports["o1"].center-tdring.ports["o2"].center
    str_td2r = sr << gf.c.straight(length=delta[0],layer = oplayer,width = width_single)
    str_td2r.connect("o1",tdring.ports["o2"])

    bend_out1 = sr << gf.c.bend_euler(width=width_near, layer=oplayer, angle=90, radius=r_ring / 2 + r_euler_false / 2)
    bend_out1.connect("o1", ring.ports["Through"])
    taper_s2n2 = sr << gf.c.taper(width1 = width_near,width2 = width_single,length=length_taper,layer=oplayer)
    taper_s2n2.connect("o1", bend_out1.ports["o2"])
    delta = bend_out1.ports["o2"].center - tetring.ports["o2"].center
    tetring.movey(delta[1])
    str_tet2r = gf.routing.get_bundle(taper_s2n2.ports["o2"],tetring.ports["o1"],layer = oplayer,width = width_single
                                                ,radius = r_euler_true)
    for route in str_tet2r:
        sr.add(route.references)
    # sr_cld1 = gf.geometry.offset(sr, distance=width_cld, layer=LAYER.CLD2)
    # sr_cld2 = gf.geometry.offset(sr_cld1, distance=-0.8, layer=LAYER.CLD2)
    # sr.add_ref(sr_cld2)
    sr.add_port("input",port=tdring.ports["o1"])
    sr.add_port("output",port=tetring.ports["o1"])
    sr.add_port("RingC",port=tetring.ports["o1"],center=ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    return sr
# %% SingleRing2_2: pulleyRing taper_s2n before output bend
@gf.cell
def SingleRing2_3(
        r_ring: float = 120,
        r_bend: float = 100,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float =5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 200,
        length_total:float = 10000,
        pos_ring:float = 500,
        gap_rc: float = 1,
        tin:Component = None,
        tout:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("Ring")
    ring = sr << RingPulley2(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        AngleCouple=angle_rc,WidthHeat=width_heat
    )
    tdring = sr << tin
    tetring = sr << tout
    ring.connect("Input",destination=tdring.ports["o2"]).movex(pos_ring).mirror_y("Input")
    delta = tetring.ports["o2"].center-tetring.ports["o1"].center
    tetring.connect("o1",tdring.ports["o1"]).mirror_x("o1").movex(length_total-delta[0])
    taper_s2n1 = sr << gf.c.taper(width1 = width_single,width2 = width_near,length=length_taper,layer=oplayer)
    taper_s2n1.connect("o2",ring.ports["Input"])
    delta = taper_s2n1.ports["o1"].center-tdring.ports["o2"].center
    str_td2r = sr << gf.c.straight(length=delta[0],layer = oplayer,width = width_single)
    str_td2r.connect("o1",tdring.ports["o2"])

    taper_s2n2 = sr << gf.c.taper(width1 = width_near,width2 = width_single,length=length_taper,layer=oplayer)
    taper_s2n2.connect("o1", ring.ports["Through"])
    bend_out1 = sr << gf.c.bend_euler(width=width_single, layer=oplayer, angle=90, radius=r_bend,p=0.5)
    bend_out1.connect("o1", destination=taper_s2n2.ports["o2"])

    delta = bend_out1.ports["o2"].center - tetring.ports["o2"].center
    tetring.movey(delta[1])
    str_tet2r = gf.routing.get_bundle(bend_out1.ports["o2"],tetring.ports["o1"],layer = oplayer,width = width_single
                                                ,radius = r_euler_true)
    for route in str_tet2r:
        sr.add(route.references)
    # sr_cld1 = gf.geometry.offset(sr, distance=width_cld, layer=LAYER.CLD2)
    # sr_cld2 = gf.geometry.offset(sr_cld1, distance=-0.8, layer=LAYER.CLD2)
    # sr.add_ref(sr_cld2)
    sr.add_port("input",port=tdring.ports["o1"])
    sr.add_port("output",port=tetring.ports["o1"])
    sr.add_port("RingC",port=tetring.ports["o1"],center=ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    return sr
# %% SingleRing3:couple angle could lager than 90,less than 180
@gf.cell
def SingleRing3(
        r_ring: float = 120,
        r_bend: float = r_euler_true,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float =5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 150,
        length_total:float = 10000,
        pos_ring:float = 500,
        gap_rc: float = 1,
        tet:Component = None,
        td:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("Ring")
    tdring = sr << td
    tetring = sr << tet
    ring = sr << RingPulley3(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        AngleCouple=angle_rc,WidthHeat=width_heat
    )
    ring.connect("Input",destination=tdring.ports["o2"]).movex(pos_ring).mirror_y("Input")
    delta = tetring.ports["o2"].center-tetring.ports["o1"].center
    #input
    tetring.connect("o1",tdring.ports["o1"]).mirror_x("o1").movex(length_total-delta[0])
    taper_s2n1 = sr << gf.c.taper(width1 = width_single,width2 = width_near,length=length_taper,layer=oplayer)
    taper_s2n1.connect("o2",ring.ports["Input"])
    delta = taper_s2n1.ports["o1"].center-tdring.ports["o2"].center
    str_td2r = sr << gf.c.straight(length=delta[0],layer = oplayer,width = width_single)
    str_td2r.connect("o1",tdring.ports["o2"])
    #output
    taper_s2n2 = sr << gf.c.taper(width1=width_near, width2=width_single, length=length_taper, layer=oplayer)
    taper_s2n2.connect("o1", ring.ports["Through"])
    bend_out2 = sr << gf.c.bend_euler(width = width_single,layer = oplayer,angle=180,radius=r_bend,p=1)
    bend_out2.connect("o1",taper_s2n2.ports["o2"])
    delta = bend_out2.ports["o2"].center-tetring.ports["o2"].center
    tetring.movey(delta[1])
    str_tet2r = gf.routing.get_bundle(bend_out2.ports["o2"],tetring.ports["o1"],layer = oplayer,width = width_single
                                                ,radius = r_euler_false)
    for route in str_tet2r:
        sr.add(route.references)
    # sr_cld1 = gf.geometry.offset(sr, distance=width_cld, layer=LAYER.CLD2)
    # sr_cld2 = gf.geometry.offset(sr_cld1, distance=-0.8, layer=LAYER.CLD2)
    # sr.add_ref(sr_cld2)
    sr.add_port("input",port=tdring.ports["o1"])
    sr.add_port("output",port=tetring.ports["o1"])
    sr.add_port("RingC",port=tetring.ports["o1"],center=ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    return sr

# %% SingleRing4: couple angle could lager than 180
@gf.cell
def SingleRing4(
        r_ring: float = 120,
        r_bend: float = r_euler_true,
        width_ring: float = 1,
        width_near: float = 2,
        width_heat:float =5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 150,
        length_total:float = 10000,
        pos_ring:float = 500,
        gap_rc: float = 1,
        tet:Component = None,
        td:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("Ring")
    tdring = sr << td
    tetring = sr << tet
    ring = sr << RingPulley4(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        AngleCouple=angle_rc,WidthHeat=width_heat
    )
    ring.connect("Input",destination=tdring.ports["o2"]).movex(pos_ring).mirror_y("Input")
    delta = tetring.ports["o2"].center-tetring.ports["o1"].center
    #input
    tetring.connect("o1",tdring.ports["o1"]).mirror_x("o1").movex(length_total-delta[0])
    taper_s2n1 = sr << gf.c.taper(width1 = width_single,width2 = width_near,length=length_taper,layer=oplayer)
    taper_s2n1.connect("o2",ring.ports["Input"])
    delta = taper_s2n1.ports["o1"].center-tdring.ports["o2"].center
    str_td2r = sr << gf.c.straight(length=delta[0],layer = oplayer,width = width_single)
    str_td2r.connect("o1",tdring.ports["o2"])
    #output
    taper_s2n2 = sr << gf.c.taper(width1=width_near, width2=width_single, length=length_taper, layer=oplayer)
    taper_s2n2.connect("o1", ring.ports["Through"])
    delta = taper_s2n2.ports["o2"].center-tetring.ports["o2"].center
    tetring.movey(delta[1])
    str_tet2r = gf.routing.get_bundle(taper_s2n2.ports["o2"],tetring.ports["o1"],layer = oplayer,width = width_single
                                                ,radius = r_euler_false)
    for route in str_tet2r:
        sr.add(route.references)
    # sr_cld1 = gf.geometry.offset(sr, distance=width_cld, layer=LAYER.CLD2)
    # sr_cld2 = gf.geometry.offset(sr_cld1, distance=-0.8, layer=LAYER.CLD2)
    # sr.add_ref(sr_cld2)
    sr.add_port("input",port=tdring.ports["o1"])
    sr.add_port("output",port=tetring.ports["o1"])
    sr.add_port("RingC",port=tetring.ports["o1"],center=ring.ports["RingL"].center/2+ring.ports["RingR"].center/2)
    return sr
# %% SingleRing5: racetrack ring
@gf.cell
def SingleRunRing(
        r_ring: float = 2000,
        r_bend: float = r_euler_true,
        width_ring: float = 8,
        width_near: float = 4,
        width_heat:float =5,
        width_single:float =1,
        angle_rc: float = 20,
        length_taper: float = 500,
        length_total:float = 10000,
        pos_ring:float = 5000,
        gap_rc: float = 1,
        tout:Component = None,
        tin:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("RunRing")
    ring = sr << RunRingPulley(
        WidthRing=width_ring,WidthNear=width_near,GapRing=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        AngleCouple=angle_rc,IsAD=False
    )
    taper_s2n_1 = sr << gf.c.taper(width1=width_single,width2=width_near,length=length_taper,layer = oplayer)
    taper_s2n_2 = sr << gf.c.taper(width2=width_single, width1=width_near, length=length_taper,layer = oplayer)
    ring.rotate(90).movex(pos_ring)
    taper_s2n_1.connect("o2",destination=ring.ports["Input"])
    taper_s2n_2.connect("o1",destination=ring.ports["Through"])
    bend_single_1 = sr << gf.c.bend_euler(width = width_single,angle = -90,radius=r_bend,layer=oplayer)
    bend_single_2 = sr << gf.c.bend_euler(width=width_single, angle=90, radius=r_bend, layer=oplayer)
    bend_single_1.connect("o2",destination=taper_s2n_1.ports["o1"])
    bend_single_2.connect("o1",destination=taper_s2n_2.ports["o2"])
    #input
    if tin != None:
        ctin = sr << tin
        ctin.connect("o2",bend_single_1.ports["o1"]).movex(-pos_ring)
        route_in = gf.routing.get_route(ctin.ports["o2"],bend_single_1.ports["o1"],layer= oplayer,width = width_single)
        sr.add(route_in.references)
    #output
    if tout != None:
        ctout = sr << tout
        ctout.connect("o2",destination=ctin.ports["o1"]).movex(length_total)
        delta = ctout.ports["o1"].center-bend_single_2.ports["o2"].center
        ctout.movey(-delta[1])
        route_out = gf.routing.get_route(ctout.ports["o1"],bend_single_2.ports["o2"],layer= oplayer,width = width_single)
        sr.add(route_out.references)
    sr.add_port("input",port=tin.ports["o1"])
    sr.add_port("output",port=tout.ports["o1"])
    sr.add_port("RingC",port=ring.ports["Input"],center=ring.ports["Rcen1"].center/2+ring.ports["Rcen2"].center/2)
    return sr
# %% SingleRunRing2_1: racetrack ring,straigh couple straight in
@gf.cell
def SingleRunRing2_1(
        r_ring: float = 2000,
        r_bend: float = r_euler_true,
        width_ring: float = 8,
        width_single:float =1,
        length_rc: float = 20,
        length_run: float = 500,
        length_taper: float = 500,
        length_updown:float = 1500,
        length_total:float = 10000,
        pos_ring:float = 5000,
        gap_rc: float = 1,
        tout:Component = None,
        tin:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("RunRing")
    width_near = width_ring
    ring = sr << RunRingPulley2(
        WidthRing=width_ring,LengthRun=length_run,GapRun=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        LengthCouple=length_rc,IsAD=False
    )
    taper_s2n_1 = sr << gf.c.taper(width1=width_single,width2=width_near,length=length_taper,layer = oplayer)
    taper_s2n_2 = sr << gf.c.taper(width2=width_single, width1=width_near, length=length_taper,layer = oplayer)
    ring.rotate(270).movex(pos_ring)
    taper_s2n_1.connect("o2",destination=ring.ports["Input"])
    taper_s2n_2.connect("o1",destination=ring.ports["Through"])
    bend_outsingle_1 = sr << gf.c.bend_euler(width = width_single,angle = -90,radius=r_bend,layer=oplayer)
    bend_outsingle_1.connect("o1",destination=taper_s2n_2.ports["o2"])
    str_outsingle_1 = sr << gf.c.straight(width = width_single,length = length_updown,layer=oplayer)
    str_outsingle_1.connect("o1",destination=bend_outsingle_1.ports["o2"])
    bend_outsingle_2 = sr << gf.c.bend_euler(width=width_single, angle=90, radius=r_bend, layer=oplayer)
    bend_outsingle_2.connect("o1",destination=str_outsingle_1.ports["o2"])
    #input
    if tin != None:
        ctin = sr << tin
        ctin.connect("o2",ring.ports["Input"]).movex(-pos_ring)
        route_in = gf.routing.get_route(ctin.ports["o2"],taper_s2n_1.ports["o1"],layer= oplayer,width = width_single)
        sr.add(route_in.references)
    #output
    if tout != None:
        ctout = sr << tout
        ctout.connect("o2",destination=ctin.ports["o1"]).movex(length_total)
        delta = ctout.ports["o1"].center-bend_outsingle_2.ports["o2"].center
        ctout.movey(-delta[1])
        route_out = gf.routing.get_route(ctout.ports["o1"],bend_outsingle_2.ports["o2"],layer= oplayer,width = width_single)
        sr.add(route_out.references)
    sr.add_port("input",port=ctin.ports["o1"])
    sr.add_port("output",port=ctout.ports["o1"])
    sr.add_port("RingC",port=ring.ports["Input"],center=ring.ports["Rcen1"].center/2+ring.ports["Rcen2"].center/2)
    add_labels_to_ports(sr)
    return sr
# %% SingleRunRing2_2: racetrack ring,straigh couple straight out
@gf.cell
def SingleRunRing2_2(
        r_ring: float = 2000,
        r_bend: float = r_euler_true,
        width_ring: float = 8,
        width_single:float =1,
        length_rc: float = 20,
        length_run: float = 200,
        length_taper: float = 500,
        length_updown:float = 1500,
        length_total:float = 10000,
        pos_ring:float = 5000,
        gap_rc: float = 1,
        tout:Component = None,
        tin:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("RunRing")
    width_near = width_ring
    ring = sr << RunRingPulley2(
        WidthRing=width_ring,LengthRun=length_run,GapRun=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        LengthCouple=length_rc,IsAD=False
    )
    taper_s2n_1 = sr << gf.c.taper(width1=width_single,width2=width_near,length=length_taper,layer = oplayer)
    taper_s2n_2 = sr << gf.c.taper(width2=width_single, width1=width_near, length=length_taper,layer = oplayer)
    ring.rotate(90).mirror_x("Input").movex(pos_ring)
    taper_s2n_1.connect("o2",destination=ring.ports["Input"])
    taper_s2n_2.connect("o1",destination=ring.ports["Through"])
    bend_insingle_1 = sr << gf.c.bend_euler(width = width_single,angle = 90,radius=r_bend,layer=oplayer)
    bend_insingle_1.connect("o2",destination=taper_s2n_1.ports["o1"])
    str_insingle_1 = sr << gf.c.straight(width = width_single,length = length_updown,layer=oplayer)
    str_insingle_1.connect("o2",destination=bend_insingle_1.ports["o1"])
    bend_insingle_2 = sr << gf.c.bend_euler(width = width_single,angle = -90,radius=r_bend,layer=oplayer)
    bend_insingle_2.connect("o2",destination=str_insingle_1.ports["o1"])
    #input
    if tin != None:
        ctin = sr << tin
        ctin.connect("o2",bend_insingle_2.ports["o1"]).movex(-pos_ring)
        route_in = gf.routing.get_route(ctin.ports["o2"],bend_insingle_2.ports["o1"],layer= oplayer,width = width_single)
        sr.add(route_in.references)
    #output
    if tout != None:
        ctout = sr << tout
        ctout.connect("o2",destination=ctin.ports["o1"]).movex(length_total)
        delta = ctout.ports["o1"].center-taper_s2n_2.ports["o2"].center
        ctout.movey(-delta[1])
        route_out = gf.routing.get_route(ctout.ports["o1"],taper_s2n_2.ports["o2"],layer= oplayer,width = width_single)
        sr.add(route_out.references)
    sr.add_port("input",port=ctin.ports["o1"])
    sr.add_port("output",port=ctout.ports["o1"])
    sr.add_port("RingC",port=ring.ports["Input"],center=ring.ports["Rcen1"].center/2+ring.ports["Rcen2"].center/2)
    add_labels_to_ports(sr)
    return sr

# %% SingleRunRing2_3: racetrack ring,straigh couple straight in & out
@gf.cell
def SingleRunRing2_3(
        r_ring: float = 2000,
        width_ring: float = 8,
        width_single:float =1,
        length_rc: float = 20,
        length_run: float = 400,
        length_taper: float = 500,
        length_total:float = 10000,
        pos_ring:float = 5000,
        gap_rc: float = 1,
        IsLabels: float = True,
        tout:Component = None,
        tin:Component = None,
        oplayer: LayerSpec = LAYER.WG,
) -> Component:
    sr = gf.Component("RunRing")
    width_near = width_ring
    Cring =  RunRingPulley2HS(
        WidthRing=width_ring,LengthRun=length_run,GapRun=gap_rc,oplayer=oplayer,RadiusRing=r_ring,
        LengthCouple=length_rc,IsAD=False,IsLabels=False,
    )
    ring = sr <<Cring[0]
    taper_s2n_1 = sr << gf.c.taper(width1=width_single,width2=width_near,length=length_taper,layer = oplayer)
    taper_s2n_2 = sr << gf.c.taper(width2=width_single, width1=width_near, length=length_taper,layer = oplayer)
    ring.rotate(90).mirror_x("Input").movex(pos_ring)
    taper_s2n_1.connect("o2",destination=ring.ports["Input"])
    taper_s2n_2.connect("o1",destination=ring.ports["Through"])
    #input
    if tin != None:
        ctin = sr << tin
        ctin.connect("o2",taper_s2n_1.ports["o1"]).movex(-pos_ring)
        route_in = gf.routing.get_route(ctin.ports["o2"],taper_s2n_1.ports["o1"],layer= oplayer,width = width_single)
        sr.add(route_in.references)
        sr.add_port("input", port=ctin.ports["o1"])
    #output
    if tout != None:
        ctout = sr << tout
        ctout.connect("o2",destination=ctin.ports["o1"]).movex(length_total)
        route_out = gf.routing.get_route(ctout.ports["o1"],taper_s2n_2.ports["o2"],layer= oplayer,width = width_single)
        sr.add(route_out.references)
        sr.add_port("output",port=ctout.ports["o1"])
    sr.add_port("RingC",port=ring.ports["Input"],center=ring.ports["Rcen1"].center/2+ring.ports["Rcen2"].center/2)
    if IsLabels:
        add_labels_to_ports(sr)
    return sr
def SingleRunRing2_3h(
        r_ring: float = 2000,
        delta_heat: float = 10,
        width_ring: float = 8,
        width_single:float =1,
        length_rc: float = 20,
        length_run: float = 400,
        length_taper: float = 500,
        length_total:float = 10000,
        pos_ring:float = 5000,
        gap_rc: float = 1,
        gap_route: float = 100,
        IsLabels: float = True,
        tout:Component = None,
        tin:Component = None,
        oplayer: LayerSpec = LAYER.WG,
        heatlayer: LayerSpec = LAYER.M1,
) -> [Component]:
    sr = gf.Component("RunRing")
    sh = gf.Component("RunRingHeat")
    width_near = width_ring
    Cring =  RunRingPulley2HS(
        WidthRing=width_ring,LengthRun=length_run,GapRun=gap_rc,oplayer=oplayer,RadiusRing=r_ring,GapRoute=gap_route,
        LengthCouple=length_rc,IsAD=False,IsLabels=False,DeltaHeat=delta_heat,heatlayer=heatlayer
    )
    heat = sh << Cring[1]
    ring = sr << Cring[0]
    taper_s2n_1 = sr << gf.c.taper(width1=width_single,width2=width_near,length=length_taper,layer = oplayer)
    taper_s2n_2 = sr << gf.c.taper(width2=width_single, width1=width_near, length=length_taper,layer = oplayer)
    ring.rotate(90).mirror_x("Input").movex(pos_ring)
    heat.connect("HeatIn",destination=ring.ports["HeatIn"])
    taper_s2n_1.connect("o2",destination=ring.ports["Input"])
    taper_s2n_2.connect("o1",destination=ring.ports["Through"])
    #input
    if tin != None:
        ctin = sr << tin
        ctin.connect("o2",taper_s2n_1.ports["o1"]).movex(-pos_ring)
        route_in = gf.routing.get_route(ctin.ports["o2"],taper_s2n_1.ports["o1"],layer= oplayer,width = width_single)
        sr.add(route_in.references)
        sr.add_port("input", port=ctin.ports["o1"])
    #output
    if tout != None:
        ctout = sr << tout
        ctout.connect("o2",destination=ctin.ports["o1"]).movex(length_total)
        route_out = gf.routing.get_route(ctout.ports["o1"],taper_s2n_2.ports["o2"],layer= oplayer,width = width_single)
        sr.add(route_out.references)
        sr.add_port("output",port=ctout.ports["o1"])
    sr.add_port("RingC",port=ring.ports["Input"],center=ring.ports["Rcen1"].center/2+ring.ports["Rcen2"].center/2)
    if IsLabels:
        add_labels_to_ports(sr)
    sr.add_port("HeatIn",port=ring.ports["HeatIn"])
    sh.add_port("HeatIn", port=heat.ports["HeatIn"])
    return [sr,sh]

# %% ExternalCavity:Proven design
@gf.cell
def ExternalCavity(
        r_ring: float = 200,
        radius_delta: float = 4,
        width_ring: float = 1,
        width_single:float= 1,
        width_single2:float = 1,
        width_near: float = 0.91,
        width_heat:float =5,
        width_route:float =20,
        width_cld: float = 3,
        angle_rc: float = 20,
        length_dc: float = 7,
        length_t_s2n: float = 200,
        length_taper: float = 200,
        length_r2r: float = 1000,
        length_bridge: float = 300,
        length_input: float = 330,
        gap_rc: float = 0.3,
        gap_dc: float = 0.5,
        gap_heat: float = 2,
        oplayer: LayerSpec = LAYER.WG,
        routelayer: LayerSpec = LAYER.M1,
        openlayer: LayerSpec = open,
        hetlayer: LayerSpec = LAYER.M1,
        slablayer: LayerSpec = LAYER.CLD,
        swglayer: LayerSpec = LAYER.WG,
) -> Component:
    ec_ref = gf.Component("ec_ref")
    offsetVC = ViaArray(Spacing=0.7,WidthVia=0.3,Row=15,Col=8,IsEn=True,Enclosure=0.5,ViaLayer=LAYER.CT,ViaEnLayers=[LAYER.CTE,hetlayer,swglayer])
    deltaVCY = offsetVC.ports["up"].center[1]-offsetVC.ports["down"].center[1]
    deltaVCX = -offsetVC.ports["left"].center[0] + offsetVC.ports["right"].center[0]
    # section and cross section
    S_near = gf.Section(width=width_near, offset=0, layer=oplayer, port_names=("o1", "o2"))
    S_heater1 = gf.Section(width=width_heat, offset=gap_heat+width_heat/2+width_near/2, layer=hetlayer)
    S_heater2 = gf.Section(width=width_heat, offset=-(gap_heat + width_heat / 2 + width_near/2), layer=hetlayer)
    S_swg = gf.Section(width = 2*width_heat+2*gap_heat+width_near,offset=0, layer=swglayer,port_names=("o1","o2"))
    S_Rup = gf.Section(width=width_route, offset=2*deltaVCY+gap_heat+width_near/2-width_route/2,layer=routelayer)
    S_Rdown = gf.Section(width=width_route, offset=-(2 * deltaVCY + gap_heat + width_near / 2 - width_route / 2),layer=routelayer)
    BusHeatRouteComp = gf.Component("BHRC")
    BHRC0 = BusHeatRouteComp << gf.c.straight(width=deltaVCX, length=deltaVCY * 3 + width_near + 2 * gap_heat,layer=routelayer)
    BHRC0.rotate(-90)
    CAP_viaup = gf.cross_section.ComponentAlongPath(component=offsetVC,spacing=50,offset = gap_heat+width_near/2+deltaVCY,padding = 0)
    CAP_viadown = gf.cross_section.ComponentAlongPath(component=offsetVC, spacing=50,offset=-gap_heat-width_near / 2)
    CAP_Routeup = gf.cross_section.ComponentAlongPath(component=BusHeatRouteComp,spacing=100,offset=-width_near/2-gap_heat-deltaVCY)
    CAP_Routedown = gf.cross_section.ComponentAlongPath(component=BusHeatRouteComp,spacing=100,padding = 50,offset=width_near/2+gap_heat-(deltaVCY * 2 + width_near + 2 * gap_heat))
    CS_near = gf.CrossSection(sections=[S_near])
    CS_heat = gf.CrossSection(sections=[S_heater1,S_heater2,S_swg],components_along_path=[CAP_viaup,CAP_viadown])
    CS_Route = gf.CrossSection(sections=[S_Rup,S_Rdown],components_along_path=[CAP_Routeup,CAP_Routedown])
    # taper near to single
    tsn = gf.Component()
    tsn = gf.c.taper(width1=width_near, width2=width_single, length=length_t_s2n, layer=oplayer)
    # ring ref
    coupler_ref = Coupler2X2(LengthCoup=length_dc, GapCoup=gap_dc, layer=oplayer, WidthSingle=width_single,LengthBridge=length_bridge)
    coupler2x2 = ec_ref << coupler_ref
    tapercoupler1 = ec_ref << gf.c.taper(width1=width_single, width2=width_near, length=length_t_s2n + 40,layer=oplayer)
    tapercoupler2 = ec_ref << gf.c.taper(width1=width_single, width2=width_near, length=length_t_s2n, layer=oplayer)
    tapercoupler1.connect("o1", destination=coupler2x2.ports["Output1"])
    tapercoupler2.connect("o1", destination=coupler2x2.ports["Output2"])
    bend_c2r = ec_ref << gf.c.bend_euler(width=width_near, angle=180, layer=oplayer, radius=r_euler_false*1.7,with_arc_floorplan=False,p=0.5)
    bend_c2r.connect("o1", tapercoupler2.ports["o2"]).mirror_y("o1")
    # str_c2r = ec_ref << gf.c.straight(width = width_near, layer = oplayer)
    # bend_c2r2 =
    ring_ref = DoubleRingPulley(
        WidthRing=width_ring, WidthNear=width_near, WidthEnd=0.2,
        LengthTaper=150, LengthR2R=length_r2r, DeltaRadius=radius_delta,
        RadiusRing=r_ring, RadiusBend0=40, GapRing=gap_rc,
        AngleCouple=angle_rc,
        oplayer=oplayer, heatlayer=hetlayer,
        Pitch=5,EndPort=[]
    )
    doublering = ec_ref << ring_ref
    doublering.connect("o1", bend_c2r.ports["o2"])
    doublering.movex(-length_r2r+r_ring*3)#.movey(-r_ring)
    c2rRoute1 = gf.routing.get_route_sbend(
        tapercoupler1.ports["o2"], doublering.ports["o2"], cross_section=CS_near
    )
    ec_ref.add(c2rRoute1.references)
    c2rRoute2 = gf.routing.get_route(bend_c2r.ports["o2"],doublering.ports["o1"],cross_section=CS_near)
    ec_ref.add(c2rRoute2.references)
    # ring to out
    bend_ringout = ec_ref << gf.c.bend_euler(width=width_near, angle=180, layer=oplayer, radius=r_euler_false,with_arc_floorplan=False)
    bend_ringout.connect("o1",doublering.ports["RingPort0"])
    bend_ringout1 = ec_ref << gf.c.bend_euler(width=width_near, angle=180, layer=oplayer, radius=r_euler_false*3.5,
                                             with_arc_floorplan=False)
    bend_ringout1.connect("o1", doublering.ports["RingPort1"])
    taper_r2o_0 = ec_ref << gf.c.taper(width1 = width_near,width2 = width_single2,layer = oplayer,length = length_taper)
    taper_r2o_0.connect("o1",bend_ringout.ports["o2"])
    taper_r2o_2 = ec_ref << gf.c.taper(width1 = width_near,width2 = width_single2,layer = oplayer,length = length_taper)
    taper_r2o_2.connect("o1",doublering.ports["RingPort2"])

    taper_r2o_1 = ec_ref << gf.c.taper(width1 = width_near,width2 = width_single2,layer = oplayer,length = length_taper)
    taper_r2o_1.connect("o1",bend_ringout1.ports["o2"])
    taper_r2o_3 = ec_ref << gf.c.taper(width1 = width_near,width2 = width_single2,layer = oplayer,length = length_taper)
    taper_r2o_3.connect("o1",doublering.ports["RingPort3"])
    ec_ref.add_port("Rout2", port=taper_r2o_2.ports["o2"],orientation=180)
    ec_ref.add_port("Rout0", port=taper_r2o_0.ports["o2"],orientation=180)
    ec_ref.add_port("Rout1", port=taper_r2o_1.ports["o2"],orientation=180)
    ec_ref.add_port("Rout3", port=taper_r2o_3.ports["o2"],orientation=180)
    ## input
    str_input = list(range(30))
    bend_input = list(range(30))
    bend_input[0] = ec_ref << gf.c.bend_euler(width=width_single, angle=-225, layer=oplayer, radius=r_euler_false,with_arc_floorplan=False)
    bend_input[0].connect("o2", coupler2x2.ports["Input2"])
    bend_input[1] = ec_ref << gf.c.bend_euler(width=width_single, angle=45, layer=oplayer, radius=r_euler_false,with_arc_floorplan=False)
    bend_input[1].connect("o2", bend_input[0].ports["o1"])
    str_input[0] = ec_ref<<gf.c.straight(width = width_single,length=length_bridge*3-100+length_taper,layer=oplayer)
    str_input[0].connect("o2", bend_input[1].ports["o1"])
    bend_input[2] = ec_ref << gf.c.bend_euler(width=width_single, angle=180, layer=oplayer, radius=r_euler_false,with_arc_floorplan=False)
    bend_input[2].connect("o2",str_input[0].ports["o1"])
    str_input[1] = ec_ref<<gf.c.straight(width = width_single,length=length_input-100+length_taper+length_bridge*2,layer=oplayer)
    str_input[1].connect("o2", bend_input[2].ports["o1"])
    ec_ref.add_port("input", port=str_input[1].ports["o1"])
    ## output
    str_output = list(range(30))
    bend_output = list(range(30))
    bend_output[0] = ec_ref << gf.c.bend_euler(width=width_single, angle=200, layer=oplayer, radius=r_euler_false,with_arc_floorplan=False)
    bend_output[0].connect("o2", coupler2x2.ports["Input1"])
    bend_output[1] = ec_ref << gf.c.bend_euler(width=width_single, angle=-20, layer=oplayer, radius=r_euler_false,with_arc_floorplan=False)
    bend_output[1].connect("o2", bend_output[0].ports["o1"])
    delta = tapercoupler1.ports["o2"].center-bend_output[1].ports["o1"].center
    str_output[0] = ec_ref<<gf.c.taper(width1 = width_single,width2 = width_single2,length=length_taper,layer=oplayer)
    str_output[0].connect("o2", bend_output[1].ports["o1"])
    ec_ref.add_port("output", port=str_output[0].ports["o1"], orientation=180)
    # heater
    # Ring Heater
    RingHeater1 = ec_ref << RingHeater(WidthRing=width_ring,RadiusRing=r_ring,GapHeat=gap_heat,WidthHeat=width_heat,ViaComp=offsetVC,NumOfVia=10)
    RingHeater1.connect("RingR",destination=doublering.ports["RingPort4"])
    RingHeater2 = ec_ref << RingHeater(WidthRing=width_ring, RadiusRing=r_ring+radius_delta,GapHeat=gap_heat,WidthHeat=width_heat,ViaComp=offsetVC)
    RingHeater2.connect("RingL",destination=doublering.ports["RingPort6"])
    # bus heater
    BusHeater = gf.Component(name="BusHeater")
    BusHeater_path = gf.path.straight(length=length_r2r-300)
    BusHeater_wg = BusHeater <<  gf.path.extrude(BusHeater_path,cross_section=CS_heat)
    BusHeater_route = BusHeater << gf.path.extrude(BusHeater_path,cross_section=CS_Route)
    BusHeater.add_port("o1",port=BusHeater_wg.ports["o1"])
    heat_cld0 = gf.geometry.offset(BusHeater_wg, distance=width_cld+0.8)
    heat_cld1 = gf.geometry.offset(heat_cld0, distance=-0.8, layer=slablayer)
    BusHeater.add_ref(heat_cld1)
    BusHeater_ref = ec_ref << BusHeater
    BusHeater_ref.connect("o1",destination=doublering.ports["r2ro1"]).movex(150)
    BusHeater_ref.mirror_x(port_name="o1")
    # mzi heater component
    MZIHeater = gf.Component(name="MZIHeater")
    MZI_path = gf.path.straight(length=length_bridge-100)
    MZIHeater_wg = MZIHeater << gf.path.extrude(MZI_path, cross_section=CS_heat)
    MZIHeater_route = MZIHeater << gf.path.extrude(MZI_path, cross_section=CS_Route)
    MZIHeater_cld0 = gf.geometry.offset(MZIHeater_wg, distance=width_cld + 0.8)
    MZIHeater_cld1 = gf.geometry.offset(MZIHeater_cld0, distance=-0.8, layer=slablayer)
    MZIHeater.add_ref(MZIHeater_cld1)
    MZIHeater.add_port("o1",port=MZIHeater_wg.ports["o1"],orientation=0)
    ## MZIheatup and down
    MZIHeaterup = ec_ref << MZIHeater
    MZIHeaterup.connect("o1",destination=coupler2x2.ports["Bridge1"])
    MZIHeaterup.movex(-100)
    MZIHeaterdown = ec_ref << MZIHeater
    MZIHeaterdown.connect("o1",destination=coupler2x2.ports["Bridge2"])
    MZIHeaterdown.movex(100)
    return ec_ref
# %% ExternalCavity: risky design
@gf.cell
def ExternalCavity2(
        r_ring: float = 200,
        radius_delta: float = 4,
        width_single:float = 1,
        width_ring: float = 1,
        width_near: float = 0.91,
        width_heat: float = 5,
        width_route: float = 20,
        width_cld: float = 3,
        angle_rc: float = 20,
        angle_rc3:float = 40,
        length_taper: float = 200,
        length_r2r: float = 50,
        gap_rc: float = 0.3,
        gap_dc: float = 0.5,
        gap_heat: float = 2,
        oplayer: LayerSpec = LAYER.WG,
        routelayer: LayerSpec = LAYER.M1,
        hetlayer: LayerSpec = LAYER.M1,
        slablayer: LayerSpec = LAYER.CLD,
        swglayer: LayerSpec = LAYER.WG,
        Crossing: Component = None,
        Name="ec2_ref"
) -> Component:
    ec2_ref = gf.Component(Name)
    if Crossing == None:
        crossing0 = Crossing_taper(WidthCross=0.8,WidthWg=width_single,LengthTaper=5,oplayer = oplayer)
    else:
        crossing0 = Crossing
    TriRing = ec2_ref << ADRAPRADR(WidthRing=width_ring,WidthNear=width_near,WidthSingle=width_single,
                                     RadiusRing=r_ring,DeltaRadius=radius_delta,LengthTaper=length_taper,LengthR2R=length_r2r,
                                     AngleCouple=angle_rc,AngleCouple3=angle_rc3,GapRing=gap_rc,CrossComp=crossing0,LengthR2C=500
    )
    # ring port taper & bend
    taperbend = gf.Component("taperbend")
    taper_n2s = taperbend << gf.c.taper(width1 = width_near,width2=width_single,layer=oplayer,length = length_taper/2)
    bend_n2s = taperbend << gf.c.bend_euler(width = width_single,layer = oplayer,radius=120,angle = 45)
    bend_n2s.connect("o1",taper_n2s.ports["o2"])
    taperbend.add_port("o1",port=taper_n2s.ports["o1"])
    taperbend.add_port("o2", port=bend_n2s.ports["o2"])
    taper_n2s_1 = ec2_ref << taperbend
    taper_n2s_1.connect("o1",TriRing.ports["r1Th"]).mirror_y("o1")
    taper_n2s_2 = ec2_ref << taperbend
    taper_n2s_2.connect("o1",TriRing.ports["r1Ad"]).mirror_y("o1")
    taper_n2s_3 = ec2_ref << taperbend
    taper_n2s_3.connect("o1",TriRing.ports["r2Th"])
    taper_n2s_4 = ec2_ref << taperbend
    taper_n2s_4.connect("o1",TriRing.ports["r2Ad"])
    # crossing bend
    bend45 = gf.c.bend_euler(width = width_single,radius=120,angle=45,layer=oplayer)
    bend45_o2 = ec2_ref << bend45
    bend45_o2.connect("o1",TriRing.ports["co2"])
    bend45_o3 = ec2_ref << bend45
    bend45_o3.connect("o1",TriRing.ports["co3"]).mirror_y("o1")
    ec2_ref.add_port("input",port=bend45_o2.ports["o2"])
    ec2_ref.add_port("output", port=bend45_o3.ports["o2"])
    # Heater
    offsetVC = ViaArray(Spacing=0.7, WidthVia=0.3, Row=15, Col=8, IsEn=True, Enclosure=0.5, ViaLayer=LAYER.CT,
                           ViaEnLayers=[LAYER.CTE, hetlayer, swglayer])
    deltaVCY = offsetVC.ports["up"].center[1] - offsetVC.ports["down"].center[1]
    deltaVCX = -offsetVC.ports["left"].center[0] + offsetVC.ports["right"].center[0]
    RingHeater1 = ec2_ref << RingHeater(WidthRing=width_ring,WidthRoute=width_route,WidthCLD=width_cld,WidthHeat=width_heat,
                                RadiusRing=r_ring,routelayer=LAYER.M1,ViaComp=offsetVC)
    RingHeater2 = ec2_ref << RingHeater(WidthRing=width_ring,WidthRoute=width_route,WidthCLD=width_cld,WidthHeat=width_heat,
                                RadiusRing=r_ring+radius_delta,routelayer=LAYER.M1,ViaComp=offsetVC)
    RingHeater3 = ec2_ref << RingHeater(WidthRing=width_ring,WidthRoute=width_route,WidthCLD=width_cld,WidthHeat=width_heat,
                                RadiusRing=r_ring+radius_delta/2,routelayer=LAYER.M1,ViaComp=offsetVC)
    RingHeater1.connect("RingR",destination=TriRing.ports["r1R"]).mirror_x("RingR")
    RingHeater2.connect("RingR", destination=TriRing.ports["r2R"])
    RingHeater3.connect("RingR", destination=TriRing.ports["r3R"])
    ec2_ref.add_port("to1",port=TriRing.ports["to1"])
    ec2_ref.add_port("to2", port=TriRing.ports["to2"])
    ec2_ref.add_port("r1Th",port=taper_n2s_1.ports["o2"])
    ec2_ref.add_port("r1Ad", port=taper_n2s_2.ports["o2"])
    ec2_ref.add_port("r2Th", port=taper_n2s_3.ports["o2"])
    ec2_ref.add_port("r2Ad", port=taper_n2s_4.ports["o2"])
    ec2_ref.add_port("r1L",port = TriRing.ports["r1L"])
    ec2_ref.add_port("r1R", port=TriRing.ports["r1R"])
    ec2_ref.add_port("r2L", port=TriRing.ports["r2L"])
    ec2_ref.add_port("r2R", port=TriRing.ports["r2R"])
    ec2_ref.add_port("r3L", port=TriRing.ports["r3L"])
    ec2_ref.add_port("r3R", port=TriRing.ports["r3R"])
    ec2_ref.add_port("co2", port=TriRing.ports["co2"])
    ec2_ref.add_port("co3", port=TriRing.ports["co3"])
    add_labels_to_ports(ec2_ref)
    return ec2_ref
# %%
# if main == "__main__":
# test = Component("0")
# test1 = RunRingPulley2HS()
# test << test1[0]
# test << test1[1]
# test.plot()
# test.show()