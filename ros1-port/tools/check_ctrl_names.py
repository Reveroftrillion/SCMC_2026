"""Verify the CTRL namespace refactor against its exact Git baseline, without ROS."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parent))
from ctrl_names import MAP, legacy_ast, legacy_text

WS=Path(__file__).resolve().parents[1]
REPO=WS.parent
BASE=MAP['base_commit']

def git(*args,input=None):
    return subprocess.check_output(['git','-c','safe.directory='+REPO.as_posix(),*args],cwd=REPO,input=input)

def xml_contract(element):
    return (element.tag,sorted(element.attrib.items()),(element.text or '').strip(),
            [xml_contract(child) for child in element])

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=True)
    errors=[]
    syntax=0
    for path in (WS/'src').rglob('*.py'):
        if '__pycache__' in path.parts:continue
        try:compile(path.read_bytes(),str(path),'exec');syntax+=1
        except Exception as exc:errors.append(str(path)+': '+str(exc))
    ast_entries=[]
    for old,new in MAP['paths'].items():
        if old.endswith('.py') or '/scripts/' in old:
            ast_entries.append((old,new))
    requests=''.join(BASE+':'+old+'\n' for old,_ in ast_entries).encode()
    stream=git('cat-file','--batch',input=requests)
    offset=0;ast_count=0
    for old,new in ast_entries:
        end=stream.index(b'\n',offset);header=stream[offset:end].split();offset=end+1
        size=int(header[2]);before=stream[offset:offset+size];offset+=size+1
        try:
            same=ast.dump(ast.parse(before))==ast.dump(legacy_ast((REPO/new).read_text(encoding='utf-8')))
            if not same:errors.append(new+': calculation AST differs beyond declared names')
            ast_count+=1
        except Exception as exc:errors.append(new+': '+repr(exc))
    packages={p.name:p for p in (WS/'src').iterdir() if (p/'package.xml').is_file()}
    for name,folder in packages.items():
        if not name.startswith('ctrl_'):errors.append('legacy package: '+name)
        if ET.parse(folder/'package.xml').getroot().findtext('name')!=name:errors.append(name+': manifest name differs')
        if 'project('+name+')' not in (folder/'CMakeLists.txt').read_text(encoding='utf-8'):
            errors.append(name+': CMake project differs')
        for path in (folder/name).rglob('*.py'):
            if path.name!='__init__.py' and not path.name.startswith('ctrl_'):
                errors.append(str(path)+': unprefixed runtime module')
    launch_count=node_count=0
    launches=list((WS/'src').rglob('*.launch'))+[WS/'ctrl_start.launch',WS/'docker/local.launch']
    for path in launches:
        root=ET.parse(path).getroot();launch_count+=1
        for node in root.iter('node'):
            pkg=node.get('pkg','');executable=node.get('type','')
            if pkg.startswith('ctrl_'):
                node_count+=1
                if pkg not in packages or not (packages[pkg]/'scripts'/executable).is_file():
                    errors.append(str(path)+': missing executable '+pkg+'/'+executable)
        for element in root.iter():
            for value in element.attrib.values():
                for pkg,relative in re.findall(r'\$\(find (ctrl_\w+)\)/([^\s\"<>]+)',value):
                    if '$(' in relative:continue
                    if pkg not in packages or not (packages[pkg]/relative).exists():
                        errors.append(str(path)+': missing launch resource '+pkg+'/'+relative)
    xml_count=0
    pairs=[(o,n) for o,n in MAP['paths'].items() if o.endswith('.launch')]
    pairs += [(p,p) for p in ('ros1-port/ctrl_start.launch','ros1-port/docker/local.launch')]
    for old,new in pairs:
        before=ET.fromstring(git('show',BASE+':'+old))
        after=ET.fromstring(legacy_text((REPO/new).read_text(encoding='utf-8')))
        if xml_contract(before)!=xml_contract(after):errors.append(new+': launch contract changed')
        xml_count+=1
    tree={}
    for line in git('ls-tree','-r','-z',BASE,'ros1-port/src').split(b'\0'):
        if line:
            meta,path=line.split(b'\t');tree[path.decode()]=meta.split()[2].decode()
    unchanged_assets=0
    for old,new in MAP['paths'].items():
        if any('/'+folder+'/' in old for folder in ('maps','routes','models','msg')):
            raw=(REPO/new).read_bytes()
            blob=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
            if blob!=tree[old]:errors.append(new+': protected asset/message bytes changed')
            unchanged_assets+=1
    protected=subprocess.run(['git','-c','safe.directory='+REPO.as_posix(),'diff','--quiet','--',
                              '2026_CTRL_competition','과거자료'],cwd=REPO).returncode==0
    if not protected:errors.append('preserved original/archive changed')
    runtime=WS/'src/ctrl_control/ctrl_control/ctrl_core.py'
    if not runtime.is_file():errors.append('CTRL control entry module missing')
    report=dict(base_commit=BASE,scope='static naming/AST/launch/resource checks; no ROS or vehicle execution',
                python_syntax_files=syntax,source_ast_files=ast_count,packages=len(packages),
                parsed_launches=launch_count,ctrl_node_declarations=node_count,
                preserved_launch_contracts=xml_count,unchanged_assets_and_messages=unchanged_assets,
                original_and_archive_unchanged=protected,errors=errors)
    (args.output/'ctrl-names-summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return bool(errors)

if __name__=='__main__':raise SystemExit(main())
