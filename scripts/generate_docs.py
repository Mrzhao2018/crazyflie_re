"""自动生成API文档"""

import os
import sys

# 创建docs目录
os.makedirs("docs", exist_ok=True)

# 生成Sphinx配置
conf_content = """
import os

project = 'AFC Swarm'
copyright = '2026, Crazyflie AFC Project'
author = 'AFC Team'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}
"""

with open("docs/conf.py", "w", encoding="utf-8") as f:
    f.write(conf_content)

print("[OK] 创建了 docs/conf.py")
