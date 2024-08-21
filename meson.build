project(
  'rpi2cam-vid',
  'c',
  'cpp',
meson_version : '>= 0.64.0',
version : '1.0.2',
default_options : ['werror=true', 'warning_level=3', 'cpp_std=c++17', 'c_std=c11', 'buildtype=release'],
license : 'BSD-2-Clause'
)

cpp_arguments = ['-pedantic', '-Wno-unused-parameter', '-faligned-new']

# Needed for file sizes > 32-bits.
cpp_arguments += '-D_FILE_OFFSET_BITS=64'

cxx = meson.get_compiler('cpp')
cpu = host_machine.cpu()
neon = get_option('neon_flags')

if cxx.get_id() == 'gcc'
    cpp_arguments += '-Wno-psabi'
endif

if cpu == 'aarch64' or neon == 'arm64'
    cpp_arguments += '-ftree-vectorize'
elif neon == 'armv8-neon'
    cpp_arguments += ['-mfpu=neon-fp-armv8', '-ftree-vectorize']
endif

libcamera_dep = dependency('libcamera', required : true)


summary({
            'location' : libcamera_dep.get_variable('libdir'),
            'version' : libcamera_dep.version()
        },
       section : 'libcamera')

rpi2cam_app_src = []
rpi2cam_app_dep = [libcamera_dep]


subdir('core')

add_project_arguments(cpp_arguments, language : 'cpp')

rpi2cam_app = library(
    'rpi2cam_app',
    rpi2cam_app_src,
    soversion : meson.project_version(),
    include_directories : include_directories('.'),
    install : true,
    name_prefix : '',
    dependencies : rpi2cam_app_dep,
)

# Install a symlink to the old library name for legacy purposes.
install_symlink('libcamera_app.so',
                install_dir: get_option('libdir'),
                pointing_to: 'rpi2cam_app.so')

subdir('apps')

summary({
            'libav encoder' : enable_libav,
        },
        bool_yn : true, section : 'Build configuration')
