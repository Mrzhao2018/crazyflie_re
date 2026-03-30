#!/bin/bash
# 构建文档脚本

echo "安装依赖..."
pip install sphinx sphinx-rtd-theme -q

echo "构建HTML文档..."
cd docs
sphinx-build -b html . _build/html

echo "文档已生成到: docs/_build/html/index.html"
