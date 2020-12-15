# coding:u8
# this allows to use "from utils import conversion" in __main__.py
import os, sys
sys.path.append(os.path.dirname(os.path.realpath(__file__))) # https://stackoverflow.com/questions/16981921/relative-imports-in-python-3
sys.path.append(os.path.dirname(os.path.realpath(__file__+'/gui'))) # 为了不需要手动修改 mainWindow.py 中的相对路径的 import 
sys.path.append(os.path.dirname(os.path.realpath(__file__+'/utils')))
sys.path.append(os.path.dirname(os.path.realpath(__file__+'/jsons')))
sys.path.append(os.path.dirname(os.path.realpath(__file__+'/acmdesignv2')))
# sys.path.append(os.path.dirname(os.path.realpath(__file__+'/newcode')))

__version__ = '1.0.9'
__description__ = 'A GUI for analysis of electric machinery'
__author__ = 'Jiahao Chen'
__author_email__ = 'horychen@qq.com'
__url__ = 'https://github.com/horychen/emachinery'
