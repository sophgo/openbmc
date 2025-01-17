project(
    'sophgo-rtc-device',
    'cpp',
    default_options: [
        'warning_level=3',
        'cpp_std=c++20'
    ],
    license: 'Apache-2.0',
    version: '0.1',
    meson_version: '>=0.57.0',
)

add_project_arguments(
    '-Wno-psabi',
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_ASIO_DISABLE_THREADS',
    language: 'cpp',
)

cpp_args = []



deps = [
  dependency('libgpiodcxx', fallback: ['libgpiod', 'gpiodcxx_dep'], default_options: ['bindings=cxx']),
  dependency('systemd'),
  dependency('sdbusplus', fallback: ['sdbusplus', 'sdbusplus_dep']),
  dependency('phosphor-logging', fallback: ['phosphor-logging', 'phosphor_logging_dep']),
  dependency('phosphor-dbus-interfaces'),


]

executable(
  'sophgo-rtc-dev',
  'src/sophgo-rtc-device.cpp',
  include_directories: include_directories('src'),
  cpp_args: cpp_args,
  dependencies: deps,
  install: true,
  install_dir: get_option('bindir'))

systemd = dependency('systemd')
if systemd.found()
  service_files = [
    'sophgo-rtc-device.service',
  ]
  foreach service_file : service_files
    install_data(
      service_file, install_dir: systemd.get_variable(pkgconfig: 'systemdsystemunitdir'))
  endforeach
endif

install_data(
  'config/rtc-device-config.json',
  install_dir: '/usr/share/sophgo-rtc-device/')
