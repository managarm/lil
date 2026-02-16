project('lil', ['c', 'cpp'],
	default_options: ['warning_level=2', 'c_std=gnu2x', 'cpp_std=gnu++23'],
	meson_version: '>=1.3.0',
)

pkgconfig = import('pkgconfig')

header_only = get_option('header_only')
build_utils = get_option('build_utils')

if not (header_only or build_utils) and (meson.get_compiler('cpp').get_id() != 'gcc' or meson.get_compiler('cpp').version().version_compare('<13'))
	error('At least GCC g++ 13 is needed to compile lil.')
endif

add_project_arguments(['-Wno-unused-function', '-Wno-unused-variable', '-Wno-unused-parameter', '-Wimplicit-fallthrough'], language: ['c', 'cpp'])
add_project_arguments(['-Werror=incompatible-pointer-types'], language: ['c'])

public_headers = files(
    'include/lil/fourcc.h',
    'include/lil/imports.h',
    'include/lil/intel.h',
    'include/lil/pch.h',
    'include/lil/vbt-types.h',
    'include/lil/vbt.h',
)

sources = files(
    'src/avi.cpp',
    'src/edid.cpp',
    'src/gmbus.cpp',
    'src/intel.cpp',
    'src/lvds.cpp',
    'src/pch.cpp',

    'src/ivy_bridge/crtc.cpp',
    'src/ivy_bridge/gtt.cpp',
    'src/ivy_bridge/interrupt.cpp',
    'src/ivy_bridge/ivb.cpp',
    'src/ivy_bridge/plane.cpp',

    'src/kaby_lake/brightness.cpp',
    'src/kaby_lake/cdclk.cpp',
    'src/kaby_lake/crtc.cpp',
    'src/kaby_lake/ddi-translations.cpp',
    'src/kaby_lake/ddi.cpp',
    'src/kaby_lake/dp-aux.cpp',
    'src/kaby_lake/dp.cpp',
    'src/kaby_lake/edp.cpp',
    'src/kaby_lake/encoder.cpp',
    'src/kaby_lake/gtt.cpp',
    'src/kaby_lake/hdmi.cpp',
    'src/kaby_lake/hpd.cpp',
    'src/kaby_lake/kbl.cpp',
    'src/kaby_lake/link-training.cpp',
    'src/kaby_lake/pci.cpp',
    'src/kaby_lake/pcode.cpp',
    'src/kaby_lake/pipe.cpp',
    'src/kaby_lake/plane.cpp',
    'src/kaby_lake/pll.cpp',
    'src/kaby_lake/setup.cpp',
    'src/kaby_lake/transcoder.cpp',
    'src/kaby_lake/watermarks.cpp',

	'src/vbt/parsing.cpp',
	'src/vbt/vbt.cpp',
)

util_sources = files(
	'utils/linux.c',
)

util_vbt_sources = util_sources + files(
	'utils/vbt/vbt.cpp',
)

include_directories = include_directories('include')
library = static_library('lil', sources, include_directories: include_directories, pic: false, install: true)
lil_dep = declare_dependency(link_with: library, include_directories: include_directories)

if not build_utils
    install_headers(public_headers, subdir: 'lil')
    pkgconfig.generate(library)
endif

if build_utils
	util_vbt = executable('vbt', util_vbt_sources,
        dependencies: lil_dep,
        install: true
    )
endif
