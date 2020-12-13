
import json
from collections import OrderedDict
import pandas
import os

# 定义全局Excel表名列表
sheetNameList = [u'基本参数', u'参数辨识']

# 写 json->excel
if __name__ == '!__main__':
    with open('machine_specification.json', 'r', encoding='utf-8') as f:
        d = json.load(f, object_pairs_hook=OrderedDict) # https://stackoverflow.com/questions/10844064/items-in-json-object-are-out-of-order-using-json-dumps/23820416
        # print(type(d))
        # print(json.dumps(d, indent=4, ensure_ascii=False))

    if False: # 显示写法
        基本参数字典 = OrderedDict()
        参数辨识字典 = OrderedDict()
        for motorName, details in d.items():
            基本参数字典[motorName] = details['基本参数']
            参数辨识字典[motorName] = details['参数辨识']
        with pandas.ExcelWriter('ACMInfo.xlsx') as writer:
            pandas.DataFrame(基本参数字典).to_excel(writer, sheet_name='基本参数')
            pandas.DataFrame(参数辨识字典).to_excel(writer, sheet_name='参数辨识')
    else:  # 隐式写法
        # 按Excel表名列表，初始化表字典
        for sheetName in sheetNameList:
            exec(f"{sheetName}字典 = OrderedDict()")
        # 按表赋值
        for motorName, details in d.items():
            for sheetName in sheetNameList:
                exec(f"{sheetName}字典[motorName] = details['{sheetName}']")
        # 写到硬盘
        with pandas.ExcelWriter('ACMInfo.xlsx') as writer:
            for sheetName in sheetNameList:
                exec(f"pandas.DataFrame({sheetName}字典).to_excel(writer, sheet_name='{sheetName}')")
    quit()


# 读：excel->json
if __name__ == '__main__':

    if False: # 显式写法
        df = pandas.read_excel(open('ACMInfo.xlsx', 'rb'), sheet_name=u'基本参数')
        基本参数字典 = df.set_index('代号(序列号)').to_dict(into=OrderedDict)

        df = pandas.read_excel(open('ACMInfo.xlsx', 'rb'), sheet_name=u'参数辨识')
        参数辨识字典 = df.set_index('代号(序列号)').to_dict(into=OrderedDict)

        写硬盘字典 = OrderedDict()
        for k, _ in 基本参数字典.items():
            # print(k)
            # 写硬盘字典[k] = { '基本参数':基本参数字典[k], 
            #                  '参数辨识':参数辨识字典[k] 
            #                 }
            写硬盘字典[k] = OrderedDict([('基本参数', 基本参数字典[k])])
            try:
                写硬盘字典[k].update(OrderedDict([('参数辨识', 参数辨识字典[k])]))
            except KeyError as e:
                print(e)
                print(f'【警告】参数辨识的表里没有{k}这款电机的数据')

    else: # 隐式写法

        # dataframe -> dict
        for sheetName in sheetNameList:
            df = pandas.read_excel(open('ACMInfo.xlsx', 'rb'), sheet_name=sheetName)
            exec(f"{sheetName}字典 = df.set_index('代号(序列号)').to_dict(into=OrderedDict)")

        写硬盘字典 = OrderedDict()
        for k, _ in 基本参数字典.items():
            print(k)
            try:
                写硬盘字典[k] = OrderedDict([ eval(f"('{sheetName}', {sheetName}字典[k])") for sheetName in sheetNameList ])
            except KeyError as e:
                print(f'\t【警告】{sheetName}的表里没有“{k}”这款电机的数据。')

    with open('machine_specification-auto.json', 'w', encoding='utf-8') as f:
        json.dump(写硬盘字典, f, ensure_ascii=False, indent=4)

class MotorJson(object):
    """docstring for MotorJson"""
    def __init__(self):
        # with open('machine_specification-auto.json', 'r', encoding='utf-8') as f:
        with open(os.path.dirname(__file__)+'/machine_specification-auto.json', 'r', encoding='utf-8') as f:
            self.d = json.load(f, object_pairs_hook=OrderedDict) # https://stackoverflow.com/questions/10844064/items-in-json-object-are-out-of-order-using-json-dumps/23820416

