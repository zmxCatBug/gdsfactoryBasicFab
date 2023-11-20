import numpy as np
from collections.abc import Callable
from functools import partial
from pydantic import BaseModel

import gdsfactory as gf
from gdsfactory.generic_tech import get_generic_pdk
from gdsfactory.pdk import get_active_pdk
from gdsfactory.component import Component
from gdsfactory.typings import Layer, LayerSpec, LayerSpecs
from gdsfactory.path import Path, _fresnel, _rotate_points
from gdsfactory.add_pins import add_pin_rectangle_inside
from gdsfactory.config import CONF, print_version_pdks, print_version_plugins
from gdsfactory.cross_section import cross_section
from gdsfactory.technology import (
    LayerLevel,
    LayerStack,
    LayerView,
    LayerViews,
    LayerMap,
)

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
    alpha = np.radians(angle*2)
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


def Ring_Pully_Allpass(
    radiusRing: float = 900,
    widthRing: float = 1,
    radiusBus: float = 906,
    widthBus: float = 9,
    angleBus: float = 9,
    angleBend: float = 2,
    radiusEulerBend: float = 500,
    yInput: float = 1200,
    yThrough: float = -1200,
    widthSingleMode: float = 4,
    lengthTaper: float = 400,
    layer: LayerSpec = "WG",
    layers: LayerSpecs | None = None,
) -> Component:
    c = gf.Component()
    layers = layers or [layer]
    
    for layer in layers:
        layer = gf.get_layer(layer)

        # ring 
        ringpath = gf.path.arc(radius=radiusRing, angle=360)
        ring = c.add_ref(gf.path.extrude(ringpath, layer=layer, width=widthRing))
        ring.center = (0,0)
        
        # coupling
        pathBus0 = gf.path.arc(radius=radiusBus, angle=angleBus, npoints=angleBus/1e-3,
                              ).move((0,-radiusBus)).rotate(-angleBus/2+135,(0,0))
        
        pathBendInner = euler_Bend_Half(radius=radiusBus, angle=angleBend, p=1, use_eff=False)
        pathBendOuter1 = gf.path.euler(radius=radiusEulerBend, angle=15-angleBend-angleBus/2, p=1, use_eff=False)
        pathBendOuter2 = gf.path.euler(radius=radiusEulerBend, angle=-(45-angleBend-angleBus/2), p=1, use_eff=False)
        
        pathEuler30 = gf.path.euler(radius=radiusEulerBend, angle=30, p=1, use_eff=False)
        pathEuler90 = gf.path.euler(radius=radiusEulerBend, angle=90, p=1, use_eff=False)
        
        pathBus1 = pathBus0 + pathBendInner + pathBendOuter1 
        pathBus2 = pathBus0.mirror((0,0),(1,1))+ pathBendInner.mirror() + pathBendOuter2 
        
        pathStraight1 = gf.path.straight((yInput-pathBus1.ymax-pathEuler30.ysize)*2)
        pathStraight2 = gf.path.straight(pathBus2.ymin-yThrough-pathEuler90.ysize)
        
        pathBus1 += pathStraight1 + pathEuler30
        pathBus2 += pathStraight2 + pathEuler90
        
        p1 = gf.path.extrude(pathBus1, layer=layer, width=widthBus)
        p2 = gf.path.extrude(pathBus2, layer=layer, width=widthBus)
        
        c.add_ref(p1.rotate(0))
        c.add_ref(p2.rotate(0))
        
        # taper to single mode waveguide
        Taper = gf.components.taper2(length=lengthTaper, width1=widthBus, width2=widthSingleMode,
                                     with_bbox=True, with_two_ports=True,  
                                     port_order_name=['o1', 'o2'], port_order_types=['optical', 'optical'], 
                                     add_pins=True)

        taper1 = c.add_ref(Taper).connect("o1", p1.ports["o2"])
        taper2 = c.add_ref(Taper).connect("o1", p2.ports["o2"])
        
        # ports: 
        c.add_port(name="layer"+str(layer)+"Input", port=taper1.ports["o2"])
        c.add_port(name="layer"+str(layer)+"Through",port=taper2.ports["o2"])
    return c


def Ring_Pully_AddDrop(
    radiusRing: float = 900,
    widthRing: float = 8,
    radiusInputBus: float = 950,
    widthInputBus: float = 2,
    angleInputBus: float = 9,
    radiusAddBus: float = 950,
    widthAddBus: float = 2,
    angleAddBus: float = 9,
    angleBend: float = 2,
    radiusEulerBend: float = 500,
    yInput: float = 1200,
    yThrough: float = -1200,
    yAdd: float = -1300,
    yDrop: float = -1250,
    widthSingleMode: float = 3,
    lengthTaper: float = 400,
    layer: LayerSpec = "WG",
    layers: LayerSpecs | None = None,
) -> Component:
    c = gf.Component()
    layers = layers or [layer]
    
    for layer in layers:
        layer = gf.get_layer(layer)

        # ring 
        ringpath = gf.path.arc(radius=radiusRing, angle=360)
        ring = c.add_ref(gf.path.extrude(ringpath, layer=layer, width=widthRing))
        ring.center = (0,0)
        
        # coupling: Input + Through
        pathInputThroughArc = gf.path.arc(radius=radiusInputBus,angle=angleInputBus, npoints=angleInputBus/0.01, 
                              ).move((0,-radiusInputBus)).rotate(-angleInputBus/2+135,(0,0))
        
        pathInputThroughBendInner = euler_Bend_Half(radius=radiusInputBus, angle=angleBend, p=1, use_eff=False)
        pathInputBendOuter = gf.path.euler(radius=radiusEulerBend, angle=15-angleBend-angleInputBus/2, 
                                       p=1, use_eff=False)
        pathThroughBendOuter = gf.path.euler(radius=radiusEulerBend, angle=-(45-angleBend-angleInputBus/2), 
                                       p=1, use_eff=False)
        
        pathInputEuler30 = gf.path.euler(radius=radiusEulerBend, angle=30, p=1, use_eff=False)
        pathThroughEuler90 = gf.path.euler(radius=radiusEulerBend, angle=90, p=1, use_eff=False)
        
        pathInputBus = pathInputThroughArc + pathInputThroughBendInner + pathInputBendOuter
        pathThroughBus = pathInputThroughArc.mirror((0,0),(1,1)) + \
            pathInputThroughBendInner.mirror() + pathThroughBendOuter
        
        pathInputStraight = gf.path.straight((yInput-pathInputBus.ymax-pathInputEuler30.ysize)*2)
        pathThroughStraight = gf.path.straight(pathThroughBus.ymin-yThrough-pathThroughEuler90.ysize)
        
        pathInputBus += pathInputStraight + pathInputEuler30
        pathThroughBus += pathThroughStraight + pathThroughEuler90
        
        pInput = gf.path.extrude(pathInputBus, layer=layer, width=widthInputBus)
        pThrough = gf.path.extrude(pathThroughBus, layer=layer, width=widthInputBus)
        
        c.add_ref(pInput.rotate(0))
        c.add_ref(pThrough.rotate(0))
        
        # taper to single mode waveguide
        Taper = gf.components.taper2(length=lengthTaper, width1=widthInputBus, width2=widthSingleMode,
                                     with_bbox=True, with_two_ports=True, 
                                     port_order_name=['o1', 'o2'], port_order_types=['optical', 'optical'], 
                                     add_pins=True)

        taperInput = c.add_ref(Taper).connect("o1", pInput.ports["o2"])
        taperThrough = c.add_ref(Taper).connect("o1", pThrough.ports["o2"])
        
        # coupling: add + drop
        pathAddDropArc = gf.path.arc(radius=radiusAddBus,angle=angleAddBus, npoints=angleInputBus/0.01, 
                              ).move((0,-radiusAddBus)).rotate(-angleAddBus/2-45,(0,0))
        
        pathAddDropBendInner = euler_Bend_Half(radius=radiusAddBus, angle=angleBend, p=1, use_eff=False)
        pathAddBendOuter = gf.path.euler(radius=radiusEulerBend/1.75, angle=195+(angleBend+angleAddBus/2), 
                                       p=1, use_eff=False)
        pathDropBendOuter = gf.path.euler(radius=radiusEulerBend, angle=15-(angleBend+angleAddBus/2), 
                                       p=1, use_eff=False)
        pathAddDropEuler30 = gf.path.euler(radius=radiusEulerBend, angle=30, p=1, use_eff=False)
        
        pathDropBus = pathAddDropArc + pathAddDropBendInner + pathDropBendOuter
        pathAddBus = pathAddDropArc.mirror((0,0),(1,1)) + pathAddDropBendInner.mirror() + \
            pathAddBendOuter 
        
        pathDropStraight = gf.path.straight((pathDropBus.ymin-yDrop-pathAddDropEuler30.ymax)*2)
        pathAddStraight = gf.path.straight((pathAddBus.points[-1][-1]-yAdd-pathAddDropEuler30.ymax)*2)
        
        pathDropBus += pathDropStraight + pathAddDropEuler30
        pathAddBus += pathAddStraight + pathAddDropEuler30
        
        pAdd = gf.path.extrude(pathAddBus, layer=layer, width=widthAddBus)
        pDrop = gf.path.extrude(pathDropBus, layer=layer, width=widthAddBus)
        
        c.add_ref(pAdd)
        c.add_ref(pDrop)
        
        taperAdd = c.add_ref(Taper).connect("o1", pAdd.ports["o2"])
        taperDrop = c.add_ref(Taper).connect("o1", pDrop.ports["o2"])
        
        # ports: 
        c.add_port(name="layer"+str(layer)+"Input", port=taperInput.ports["o2"])
        c.add_port(name="layer"+str(layer)+"Through",port=taperThrough.ports["o2"])
        c.add_port(name="layer"+str(layer)+"Add", port=taperAdd.ports["o2"])
        c.add_port(name="layer"+str(layer)+"Drop",port=taperDrop.ports["o2"])
        
    return c


def SpiralRing_Straight_Allpass(
    NSpiral: int = 12,
    widthSprialWaveguide: float = 1,
    spacingSpiralWaveguide: float = 50.0,
    radiusSpiralBend: float = 150,
    spacingSpiralCenter: float = 4e3,
    spacingCoupler: float = 8.0,
    lengthCoupler: float = 200,
    widthCouplerWaveguide: float = 3,
    radiusCouplerBend: float = 30, 
    widthSingleMode: float = 4,
    lengthTaper: float = 400,
    layer: LayerSpec = "WG",
    layers: LayerSpecs | None = None,
    **kwargs
) -> Component:
    c = gf.Component()
    layers = layers or [layer]
    
    for layer in layers:
        spiralRingCrossSection = partial(gf.cross_section.strip, width=widthSprialWaveguide, layer=layer)
        # two spiral
        Spiral = gf.components.spiral_external_io(N=NSpiral, radius=radiusSpiralBend, 
                                                  cross_section=spiralRingCrossSection(), 
                                                  xspacing=spacingSpiralWaveguide, yspacing=spacingSpiralWaveguide, 
                                                  **kwargs)
        spiral1 = c.add_ref(Spiral).rotate(90)
        spiral2 = c.add_ref(Spiral).mirror((0,0)).rotate(90).movex(spacingSpiralCenter)
        
        # connect spirals
        length1 = spiral1.ports['o1'].center-spiral2.ports['o1'].center
        straight1 = c.add_ref(gf.components.straight(length=np.abs(length1[0]), 
                                                     cross_section=spiralRingCrossSection()))
        straight1.connect("o1", spiral1.ports["o1"])

        length2 = spiral1.ports['o2'].center-spiral2.ports['o2'].center
        straight2 = c.add_ref(gf.components.straight(length=np.abs(length2[0]), 
                                                     cross_section=spiralRingCrossSection()))
        straight2.connect("o1", spiral1.ports["o2"])

        # coupler: straight + 2*bend_euler_s
        couplerCrossSection = partial(gf.cross_section.strip, width=widthCouplerWaveguide, layer=layer)
        couplerCenter = straight2.center + [0,spacingCoupler]
        straightCoupler = c.add_ref(gf.components.straight(length=lengthCoupler, 
                                                     cross_section=couplerCrossSection()))
        straightCoupler.center = couplerCenter

        EulerBend = gf.components.bend_euler_s(radius=radiusCouplerBend,p=1,with_arc_floorplan=False,cross_section=couplerCrossSection())
        couplerBend1 = c.add_ref(EulerBend).mirror()
        couplerBend1.connect("o1", straightCoupler.ports["o1"])
        couplerBend2 = c.add_ref(EulerBend)
        couplerBend2.connect("o1", straightCoupler.ports["o2"])
        
        # taper to single mode waveguide
        Taper = gf.components.taper2(length=lengthTaper, width1=widthCouplerWaveguide, width2=widthSingleMode,
                                     with_bbox=True, with_two_ports=True, cross_section='xs_sc', 
                                     port_order_name=['o1', 'o2'], port_order_types=['optical', 'optical'], 
                                     add_pins=True)
        taper1 = c.add_ref(Taper).connect("o1", couplerBend1.ports["o2"])
        taper2 = c.add_ref(Taper).connect("o1", couplerBend2.ports["o2"])
        
        # ports: 
        c.add_port(name="layer"+str(layer)+"Input", port=taper1.ports["o2"])
        c.add_port(name="layer"+str(layer)+"Through",port=taper2.ports["o2"])
        
    c.info["ringLength"] = Spiral.info["length"]*2 + np.abs(length2[0]) + np.abs(length1[0])
    
    return c


def Spiral_Square_Single(
    radiusBend: float = 50,
    spacingWaveguide: float = 9.7, 
    numberLoops: int = 25,
    lengthStraightX: float = 276,
    lengthStraightY: float = 0,
    widthWaveguide: float = 1,
    widthCrossingTaper: float = 3,
    lengthCrossingTaper: float = 3,
    layer: LayerSpec = "WG",
    layers: LayerSpecs | None = None,
)-> Component:
    c = gf.Component()
    layers = layers or [layer]
    
    for layer in layers:
        layer = gf.get_layer(layer)
        # crossing component
        crossing = gf.components.crossing_from_taper(taper=gf.components.taper(
            width1=widthWaveguide, width2=widthCrossingTaper,length=lengthCrossingTaper, layer=layer))
        # spiral bend component
        pathSpiralBendEuler90 = gf.path.euler(radius=radiusBend, angle=90, p=1)
        
        # create spiral loop
        path = gf.path.straight(0)
        for i in range(numberLoops):
            path += pathSpiralBendEuler90 + gf.path.straight(lengthStraightY+spacingWaveguide*i*2) \
                        + pathSpiralBendEuler90 + gf.path.straight(lengthStraightX+spacingWaveguide*i*2)
            # add crossing
            _crossing = c.add_ref(crossing)
            _crossing.movey(path.points[-1][-1])
            _crossing.movex(-lengthStraightX/2)
            
            if i != numberLoops-1:
                path += pathSpiralBendEuler90 + gf.path.straight(lengthStraightY+spacingWaveguide*(i*2+1)) \
                            + pathSpiralBendEuler90 + gf.path.straight(lengthStraightX+spacingWaveguide*(i*2+1))
        
        p = gf.path.extrude(path, layer=layer, width=widthWaveguide)
        c.add_ref(p.rotate(0))
        
        # Input & Output Waveguide
        BendEuler90 = gf.components.bend_euler(angle=-90, width=widthWaveguide, radius=radiusBend, 
                                    p=1, with_arc_floorplan=False, direction='ccw', add_pins=True,npoints=720)
        
        straight1=c.add_ref(gf.components.straight(length=abs(lengthStraightX/2-BendEuler90.xsize+widthWaveguide/2), 
                                           width=widthWaveguide)).connect("o1", p.ports["o1"])
        
        bendEuler90p1=c.add_ref(BendEuler90).connect("o1", straight1.ports["o2"])
        
        straight2=c.add_ref(gf.components.straight(
                    length=lengthStraightY+spacingWaveguide*(numberLoops)+BendEuler90.xsize,
                    width=widthWaveguide)).connect("o1", bendEuler90p1.ports["o2"])
        
        bendEuler90p2=c.add_ref(BendEuler90).connect("o1", straight2.ports["o2"])
        
        # ports: 
        c.add_port(name="o1", port=p.ports["o2"])
        c.add_port(name="o2",port=bendEuler90p2.ports["o2"])
        
    return c


def EdgeCoupler_InverseTaper(
    widthSingleMode: float = 3,
    widthEdge: float = 0.5,
    lengthTaper: float = 400,
    lengthEdge: float = 200,
    layer: LayerSpec = "WG",
    layers: LayerSpecs | None = None,
)-> Component:
    c = gf.Component()
    layers = layers or [layer]
    
    for layer in layers:
        layer = gf.get_layer(layer)
        Taper = gf.components.taper2(length=lengthTaper, width1=widthEdge, width2=widthSingleMode,
                                     with_bbox=True, with_two_ports=True,  
                                     port_order_name=['o1', 'o2'], port_order_types=['optical', 'optical'], 
                                     add_pins=True)
        inverseTaper = c.add_ref(Taper)
        edgeStraightWaveguide = c.add_ref(gf.components.straight(
            length=lengthEdge, width=widthEdge)).connect("o1", inverseTaper.ports["o1"])
        
        c.add_port(name="o1", port=edgeStraightWaveguide.ports["o2"])
        c.add_port(name="o2",port=inverseTaper.ports["o2"])
        
    return c


def EdgeCoupler_InverseTaper_Tilt(
    widthSingleMode: float = 3,
    widthEdge: float = 0.5,
    lengthTaper: float = 400,
    radiusEulerEdge: float = 400,
    angleTilt: float = 10.0,
    layer: LayerSpec = "WG",
    layers: LayerSpecs | None = None,
)-> Component:
    c = gf.Component()
    layers = layers or [layer]
    
    for layer in layers:
        layer = gf.get_layer(layer)
        Taper = gf.components.taper2(length=lengthTaper, width1=widthEdge, width2=widthSingleMode,
                                     with_bbox=True, with_two_ports=True,  
                                     port_order_name=['o1', 'o2'], port_order_types=['optical', 'optical'], 
                                     add_pins=True)
        inverseTaper = c.add_ref(Taper)
        edgeEulerBend = c.add_ref(gf.components.bend_euler(angle=angleTilt, p=1, radius=radiusEulerEdge, 
                                                           with_arc_floorplan=False, 
                                                           direction='cw', width=widthEdge)
                                 ).connect("o1", inverseTaper.ports["o1"])
        

        c.add_port(name="o1", port=edgeEulerBend.ports["o2"])
        c.add_port(name="o2",port=inverseTaper.ports["o2"])
        
    return c

"""
define PDK
"""
gf.config.rich_output()
nm = 1e-3

class LayerMapFabBasic(LayerMap):
    WG: Layer = (15, 0)

LAYER = LayerMapFabBasic()

class LayerViewsFabBasic(LayerViews):
    WG: LayerView = LayerView(color="pink")

LAYER_VIEWS = LayerViewsFabBasic(layer_map=dict(LAYER))

def get_layer_stack_FabBasic(
    thickness_wg: float = 100 * nm
) -> LayerStack:
    """Returns FabBasic LayerStack"""
    return LayerStack(
        layers=dict(
            strip=LayerLevel(
                layer=LAYER.WG,
                thickness=thickness_wg,
                zmin=0.0,
                material="Si3N4",
            ),
        )
    )

LAYER_STACK = get_layer_stack_FabBasic()

generic_pdk = get_generic_pdk()
cells = dict(Ring_Pully_Allpass=Ring_Pully_Allpass,
             Ring_Pully_AddDrop=Ring_Pully_AddDrop,
             SpiralRing_Straight_Allpass=SpiralRing_Straight_Allpass,
             Spiral_Square_Single=Spiral_Square_Single,
             EdgeCoupler_InverseTaper=EdgeCoupler_InverseTaper,
             EdgeCoupler_InverseTaper_Tilt=EdgeCoupler_InverseTaper_Tilt
             )

fabBasic = gf.Pdk(
    name="FabBasic",
    cells=cells ,
    layers=dict(LAYER),
    base_pdk=generic_pdk,
    layer_views=LAYER_VIEWS,
    layer_stack=LAYER_STACK,
)
fabBasic.activate()

if __name__=='__main__':
    gds = gf.Component()

    ringAllPass = [1,1,1,1,1,1,1,1,1,1]
    for i in range(10):
        ringAllPass[i] = gds.add_ref(Ring_Pully_Allpass(angleBus = 6*(i+1)))
        ringAllPass[i].movex(3000*i)

    ringAddDrop = [1,1,1,1,1,1,1,1,1,1]
    for i in range(10):
        ringAddDrop[i] = gds.add_ref(Ring_Pully_AddDrop(angleInputBus = 6*(i+1), angleAddBus = 6*(i+1), angleBend=3.2))
        ringAddDrop[i].movex(3000*i)
        ringAddDrop[i].movey(3000)

    inverseTaper = gds.add_ref(EdgeCoupler_InverseTaper())
    inverseTaper.movey(4000)

    inverseTaperTilt = gds.add_ref(EdgeCoupler_InverseTaper_Tilt())
    inverseTaper.movey(4050)

    spiralRing = gds.add_ref(SpiralRing_Straight_Allpass())
    spiralRing.movey(6000)

    spiral_Square_Single = gds.add_ref(Spiral_Square_Single())
    spiral_Square_Single.movey(-2000)

    gds.write_gds("demo.gds",latten_invalid_refs=True)
    gds.plot()