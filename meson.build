project('lil', 'c')

add_project_arguments(['-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-parameter'], language: 'c')

c_std = 'gnu2x'

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
    'src/coffee_lake/crtc.c',
    'src/coffee_lake/dp.c',
    'src/coffee_lake/gtt.c',
    'src/coffee_lake/plane.c',
)

library = static_library('lil', sources, pic: false)

dependency = declare_dependency(link_with: library)
