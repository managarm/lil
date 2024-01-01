project('lil', ['c', 'cpp'],
	default_options: ['warning_level=2', 'c_std=gnu2x', 'cpp_std=gnu++23'],
	meson_version: '>=1.3.0',
)

header_only = get_option('header_only')
build_utils = get_option('build_utils')

if not (header_only or build_utils) and (meson.get_compiler('cpp').get_id() != 'gcc' or meson.get_compiler('cpp').version().version_compare('<13'))
	error('At least GCC g++ 13 is needed to compile lil.')
endif

add_project_arguments(['-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-parameter', '-Wimplicit-fallthrough'], language: ['c', 'cpp'])
add_project_arguments(['-Werror=incompatible-pointer-types'], language: ['c'])

sources = files(
    'src/avi.cpp',
    'src/edid.cpp',
    'src/gmbus.cpp',
    'src/intel.cpp',
    'src/lvds.cpp',
    'src/pch.cpp',

    'src/ivy_bridge/crtc.c',
    'src/ivy_bridge/gtt.c',
    'src/ivy_bridge/interrupt.c',
    'src/ivy_bridge/ivb.c',
    'src/ivy_bridge/plane.c',

    'src/kaby_lake/brightness.cpp',
    'src/kaby_lake/cdclk.cpp',
    'src/kaby_lake/dp-aux.cpp',
    'src/kaby_lake/gtt.cpp',
    'src/kaby_lake/pci.cpp',
    'src/kaby_lake/pcode.cpp',

	'src/vbt/parsing.cpp',
	'src/vbt/vbt.cpp',
)

util_vbt_sources = files(
	'utils/vbt/vbt.cpp',
)

include_directories = include_directories('include')

if header_only
	lil_dep = declare_dependency(include_directories: include_directories)
else
	library = static_library('lil', sources, include_directories: include_directories, pic: false, install: not header_only)
	lil_dep = declare_dependency(link_with: library, include_directories: include_directories)
endif

if build_utils
	util_vbt = executable('vbt', util_vbt_sources, dependencies: lil_dep, native: true, install: true)
endif
