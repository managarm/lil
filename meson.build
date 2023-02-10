project('lil', ['c'], default_options: ['warning_level=2'])

add_project_arguments(['-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-parameter', '-Wimplicit-fallthrough'], language: 'c')

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

include_directories = include_directories('include')
library = static_library('lil', sources, include_directories: include_directories, pic: false)
dependency = declare_dependency(link_with: library, include_directories: include_directories)
