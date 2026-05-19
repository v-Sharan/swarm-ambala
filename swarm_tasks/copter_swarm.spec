# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_all

datas = [('swarm_tasks\\envs\\worlds', 'swarm_tasks\\envs\\worlds')]
binaries = []
hiddenimports = ['swarm_tasks.utils', 'swarm_tasks.envs', 'swarm_tasks.simulation.simulation', 'swarm_tasks.controllers.command', 'swarm_tasks.controllers.base_control', 'swarm_tasks.modules.dispersion', 'swarm_tasks.modules.exploration', 'swarm_tasks.modules.formations.line', 'swarm_tasks.modules.formations.circle', 'swarm_tasks.utils.weight_functions', 'locatePosition', 'geopy.distance', 'geopy.point', 'simplekml', 'matplotlib.pyplot', 'numpy', 'mavproxy', 'bezier_curve', 'groupsplitauto', 'bezier_curve_multiple', 'groupsplitspecific', 'netifaces']
tmp_ret = collect_all('dronekit')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
tmp_ret = collect_all('pymavlink')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]
tmp_ret = collect_all('lxml')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]


block_cipher = None


a = Analysis(['swarm_tasks\\Examples\\basic_tasks\\copter_swarm.py'],
             pathex=['.', 'D:\\nithya'],
             binaries=binaries,
             datas=datas,
             hiddenimports=hiddenimports,
             hookspath=[],
             hooksconfig={},
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)

exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,  
          [],
          name='copter_swarm',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          upx_exclude=[],
          runtime_tmpdir=None,
          console=True,
          disable_windowed_traceback=False,
          target_arch=None,
          codesign_identity=None,
          entitlements_file=None )
