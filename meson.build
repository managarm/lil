project('lil', 'c')

sources = files(
    'src/edid.c',
    'src/gmbus.c',
    'src/intel.c',
    'src/lvds.c',
    'src/pci.c',
    
    'src/ivy_bridge/crtc.c',
    'src/ivy_bridge/gtt.c',
    'src/ivy_bridge/interrupt.c',
    'src/ivy_bridge/ivb.c',
    'src/ivy_bridge/plane.c',

    'src/coffee_lake/cfl.c',
    'src/coffee_lake/dp.c',
)

library = static_library('lil', sources, pic: false)

dependency = declare_dependency(link_with: library)
