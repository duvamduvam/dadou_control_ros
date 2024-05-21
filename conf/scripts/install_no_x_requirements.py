import os
import subprocess

def has_gui():
    return os.environ.get('DISPLAY') is not None

def install_requirements(file):
    subprocess.check_call([os.sys.executable, '-m', 'pip', 'install', '-r', file])

if __name__ == "__main__":
    if not has_gui():
        print("Server X is absent, installing headless dependencies...")
        install_requirements('../requirement-gpio.txt')
    else:
        print("Server X is present, no need to install headless dependencies.")
