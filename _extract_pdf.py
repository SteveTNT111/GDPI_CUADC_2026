import sys
sys.path.insert(0, r'C:\Users\BG7OEV\AppData\Roaming\Python\Python312\site-packages')
from pypdf import PdfReader

pdf_path = r'D:\文档\14_OBSIDIAN智能数据综合管理系统\02_知识库\竞赛资料\CUADC_2026\附件1-13\附件1：2026中国大学生飞行器设计创新大赛竞赛规则.pdf'

reader = PdfReader(pdf_path)
print(f'总页数: {len(reader.pages)}')

for i in range(len(reader.pages)):
    text = reader.pages[i].extract_text()
    print(f'\n{"="*60}')
    print(f'=== 第{i+1}页 ===')
    print(f'{"="*60}')
    if text and text.strip():
        print(text)
    else:
        print('(本页无文本内容，可能为图片)')
