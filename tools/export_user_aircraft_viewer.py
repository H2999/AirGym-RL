"""Export a self-contained folder with continuous motor joints for web viewing.

The training URDF is untouched. This is a kinematic illustration, not a motor
simulation: each motor/prop visual rotates together around its existing Z axis.
"""
import argparse
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import zipfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',type=Path,default=Path('airgym/assets/robots/user160'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();source=args.robot.resolve();output=args.output.resolve()
    if output==source or source in output.parents:
        raise ValueError('Viewer export must be separate from the training asset directory')
    tree=ET.parse(source/'model.urdf');root=tree.getroot()
    root.set('name','user160_motor_viewer_only')
    for i in range(1,5):
        joint=root.find(f"./joint[@name='base_link2prop_{i}']")
        if joint is None:raise ValueError(f'Missing motor joint {i}')
        joint.set('type','continuous');joint.attrib.pop('dont_collapse',None)
        axis=joint.find('axis')
        if axis is None:axis=ET.SubElement(joint,'axis')
        axis.set('xyz','0 0 -1' if i<=2 else '0 0 1')
    output.mkdir(parents=True,exist_ok=True)
    mesh_files=set()
    for mesh in root.findall('.//mesh'):
        relative=Path(mesh.get('filename'))
        if relative.is_absolute() or '..' in relative.parts:raise ValueError('Expected a local relative mesh reference')
        source_mesh=source/relative;destination=output/relative
        if not source_mesh.is_file():raise FileNotFoundError(source_mesh)
        destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source_mesh,destination);mesh_files.add(relative)
    urdf=output/'user160_motor_viewer.urdf'
    ET.indent(tree);tree.write(urdf,encoding='utf-8',xml_declaration=True)
    readme=output/'README.txt'
    readme.write_text(
        '网页电机旋转展示版本，不能用于当前 AirGym 训练。\n'
        '打开 https://viewer.robotsfan.com/ ，使用文件夹导入/拖入整个 user160_web 文件夹。\n'
        '加载 user160_motor_viewer.urdf，调节 base_link2prop_1～4 关节角度，检查桨叶旋转轴。\n'
        '必须同时提供 meshes 目录。ZIP 请先解压；网站文件选择器未列出 ZIP 支持。\n'
        '电机与桨叶外形一起旋转，仅用于运动学查看，不验证转速、推力或飞行能力。\n',encoding='utf-8')
    archive=output.with_suffix('.zip')
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as z:
        for relative in sorted(mesh_files|{Path(urdf.name),Path(readme.name)}):
            z.write(output/relative,Path(output.name)/relative)
    print('Viewer URDF:',urdf)
    print('Mesh files:',len(mesh_files))
    print('ZIP:',archive,'(extract before importing the folder)')


if __name__=='__main__':main()
