import os
import sys

# 让 `src.xxx` 风格的绝对导入可用：将仓库根加入 path。
# 源码内部使用相对导入（``from ..runtime.xxx import ...``），
# 这要求 ``src`` 本身被识别为 package 的父目录，因此指向仓库根更稳。
sys.path.insert(0, os.path.abspath('..'))

# 只 mock 未安装或需要 C++ 工具链才能 build 的依赖：
# - ``cflinkcpp``：可选 C++ radio 驱动，Windows 上装它需要 MSVC Build Tools
# 其他依赖（cflib / numpy / scipy / cvxpy / matplotlib / PyYAML）已安装，autodoc 直接真实导入；
# 强制 mock 已装包会让 ``np.ndarray | None`` 这类 PEP 604 annotation 在 class body 内 eval 失败。
autodoc_mock_imports = [
    'cflinkcpp',
]

project = 'AFC Swarm'
copyright = '2026, Crazyflie AFC Project'
author = 'AFC Team'
release = '2026-04-19'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.todo',
    'myst_parser',
    'sphinxcontrib.mermaid',
]

language = 'zh_CN'
templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'superpowers']
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_title = 'Crazyflie AFC Swarm'
myst_heading_anchors = 3
myst_fence_as_directive = ['mermaid']

autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}

todo_include_todos = True
