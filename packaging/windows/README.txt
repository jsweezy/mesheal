Mesheal for Windows (64-bit)
============================

This folder runs Mesheal without installing it. To install it like other Windows programs
(Start menu entry, open .stl files with Mesheal, uninstall from Settings > Apps), download
Mesheal-windows-x64-setup.exe from https://github.com/jsweezy/mesheal/releases instead.

To use this folder:

1. Keep the whole folder together somewhere permanent, for example in Documents. mesheal.exe
   needs the .dll files next to it.
2. Double-click mesheal.exe. If Windows shows "Windows protected your PC", click More info,
   then Run anyway. (Mesheal is not code-signed; this is asked only the first time.)
3. Optional: right-click mesheal.exe > Show more options > Pin to Start, or Send to >
   Desktop (create shortcut).
4. Optional, to open .stl files with it: right-click an .stl file > Open with > Choose another
   app > Choose an app on your PC, pick mesheal.exe in this folder, then click Always.

Command line: open PowerShell in this folder and run, for example

    .\mesheal.exe part.stl --fix-all
    .\mesheal.exe --help

To remove Mesheal, delete this folder. Its settings are in %APPDATA%\mesheal.

Documentation: https://jsweezy.github.io/mesheal/
License: GNU GPL v3 (LICENSE.md); third-party licenses are in the licenses folder.
