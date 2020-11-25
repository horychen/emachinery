# this allows to use "from utils import conversion" in __main__.py
import os, sys; sys.path.append(os.path.dirname(os.path.realpath(__file__))) # https://stackoverflow.com/questions/16981921/relative-imports-in-python-3

__version__ = '1.0.3'
__description__ = 'A GUI for analysis of electric machinery'
__author__ = 'Jiahao Chen'
__author_email__ = 'horychen@qq.com'
__url__ = 'https://github.com/horychen/emachinery'
