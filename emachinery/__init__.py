# coding:u8
# this allows to use "from utils import conversion" in __main__.py
import os, sys

# print(
#     os.path.dirname(os.path.realpath(__file__)),
#     os.path.dirname(os.path.realpath(__file__))+'/guiv2',
#     os.path.dirname(os.path.realpath(__file__))+'/utils',
#     os.path.dirname(os.path.realpath(__file__))+'/jsons',
#     os.path.dirname(os.path.realpath(__file__))+'/acmdesignv2',
#     os.path.dirname(os.path.realpath(__file__))+'/acmsimpyv1',
#     sep='\n'
# )

# 为了不需要手动修改 mainWindow.py 中的相对路径的 import 
# https://stackoverflow.com/questions/16981921/relative-imports-in-python-3
sys.path.append( os.path.dirname(os.path.realpath(__file__))                )
sys.path.append( os.path.dirname(os.path.realpath(__file__))+'/guiv2'       )
sys.path.append( os.path.dirname(os.path.realpath(__file__))+'/guiv3'       )
sys.path.append( os.path.dirname(os.path.realpath(__file__))+'/utils'       )
sys.path.append( os.path.dirname(os.path.realpath(__file__))+'/jsons'       )
sys.path.append( os.path.dirname(os.path.realpath(__file__))+'/acmdesignv2' )
sys.path.append( os.path.dirname(os.path.realpath(__file__))+'/acmsimpyv1'  )
sys.path.append( os.path.dirname(os.path.realpath(__file__))+'/tools'  )

__version__ = '2.1.0'
__description__ = 'A GUI for analysis of electric machinery'
__author__ = 'Jiahao Chen'
__author_email__ = 'horychen@qq.com'
__url__ = 'https://github.com/horychen/emachinery'


# import emachinery.gui.main
# import emachinery.acmsimcv5.CJHAcademicPlot.__cjhAcademicPlotSettings
# import emachinery.acmsimcv5.CopySimToExp
