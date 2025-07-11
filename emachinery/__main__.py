# from gui.main import main

# from emachinery.guiv2.main import startApp as main

import os
def main():
    os.system('python '+os.path.dirname(__file__) + '/main.py')

if __name__ == '__main__':
    main()
    # 这里不要写任何其他代码了，emy只会调用本文件中import进来的main()函数
