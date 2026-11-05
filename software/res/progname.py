Import("env")

import subprocess

def get_git_rev():
    git_rev = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode().strip()
    git_status = subprocess.check_output(["git", "status", "-s", "--untracked-files=no"]).decode()
    suffix = "dirty" if git_status else ""

    return git_rev + suffix

def get_version():
    with open("src/OC_version.h") as file:
        last_line = file.readlines()[-1].strip().replace('"', '')

    return last_line 

tag = get_git_rev()

build_flags = env.ParseFlags(env['BUILD_FLAGS'])
defines = build_flags.get("CPPDEFINES")

def is_loom():
    return any((d[0] if isinstance(d, (list, tuple)) else d) == "LOOM_ONLY" for d in (defines or []))

if is_loom():
    # Loom: a build number that only goes up (the count of commits), so the
    # .hex files sort by age; the commit itself stays in OC_BUILD_HASH
    count = subprocess.check_output(["git", "rev-list", "--count", "HEAD"]).decode().strip()
    env.Append(BUILD_FLAGS=[ f'-DOC_BUILD_HASH=\\"{tag}\\"' ])
    tag = count + ("dirty" if tag.endswith("dirty") else "")

env.Append(BUILD_FLAGS=[ f'-DOC_BUILD_TAG=\\"{tag}\\"' ])

if "USB_AUDIO" in defines:
    tag += "+audio"
if "USB_MTPDISK" in defines:
    tag += "+MTP"

if "T41" not in env['PIOENV']:
    version = get_version()
    for item in defines:
        if item[0] == 'OC_VERSION_EXTRA':
            version += item[1].strip('"')
    env.Replace(PROGNAME=f"o_C-phazerville-{version}-{tag}")
