release = ["-DCMAKE_BUILD_TYPE=Release", "-DUSE_LP=YES"]
debug = ["-DCMAKE_BUILD_TYPE=Debug", "-DUSE_LP=YES"]
releasenolp = ["-DCMAKE_BUILD_TYPE=Release", "-DUSE_LP=NO"]
debugnolp = ["-DCMAKE_BUILD_TYPE=Debug", "-DUSE_LP=NO"]
minimal = ["-DCMAKE_BUILD_TYPE=Release", "-DDISABLE_PLUGINS_BY_DEFAULT=YES"]

DEFAULT = "release"
DEBUG = "debug"
