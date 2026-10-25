"""Explicit CTRL naming map; original snapshots keep their original namespace."""
import ast
import json
from pathlib import Path
import re

WS = Path(__file__).resolve().parents[1]
_map_file = WS / 'tools/ctrl_name_map.json'
if not _map_file.exists():
    _map_file = Path(__file__).with_name('ctrl_name_map.json')
MAP = json.loads(_map_file.read_text(encoding='utf-8'))
PACKAGES = MAP['packages']
MODULES = MAP['modules']
PATHS = MAP['paths']
SCRIPTS = MAP['scripts']
LAUNCHES = MAP['launches']
REVERSE_MODULES = {v:k for k,v in MODULES.items()}
REVERSE_PACKAGES = {v:k for k,v in PACKAGES.items()}
MODULE_PARTS = {v.rsplit('.',1)[-1]:k.rsplit('.',1)[-1] for k,v in MODULES.items()}
FILES = {Path(a).name:Path(b).name for a,b in PATHS.items() if Path(a).name != Path(b).name}

def _replace(text, mapping):
    if not mapping: return text
    pattern = r'(?<![A-Za-z0-9_])(' + '|'.join(re.escape(k) for k in sorted(mapping,key=len,reverse=True)) + r')(?![A-Za-z0-9_])'
    return re.sub(pattern,lambda m:mapping[m[0]],text)

def package_name(name):
    return PACKAGES.get(name,name)

def module_name(name):
    if name in MODULES.values():return name
    parts=name.split('.')
    parts[0]=REVERSE_PACKAGES.get(parts[0],parts[0])
    old='.'.join(parts)
    return MODULES.get(old,_replace(old,PACKAGES))

def executable_name(name):
    return SCRIPTS.get(name,name)

def launch_name(name):
    return LAUNCHES.get(name,name)

def current_relative(relative):
    name = Path(relative).as_posix()
    prefixed = name if name.startswith('ros1-port/') else 'ros1-port/' + name
    parts = [REVERSE_PACKAGES.get(p,p) for p in Path(prefixed).parts]
    value = PATHS.get(Path(*parts).as_posix(),prefixed)
    return value if name.startswith('ros1-port/') else value[len('ros1-port/'):]

def current_path(path):
    path=Path(path)
    try: relative=path.relative_to(WS)
    except ValueError: return path
    return WS/current_relative(relative)

def ctrl_text(text):
    """Translate current launch/packaging output while reading the original unchanged."""
    urls=[]
    def protect(match):
        urls.append(match[0]);return '@@UPSTREAM_URL_'+str(len(urls)-1)+'@@'
    text=re.sub(r'https?://[^\s\"\'<>]+',protect,text)
    text=_replace(text,MODULES)
    text=_replace(text,FILES)
    for old,new in SCRIPTS.items():
        text=text.replace('/scripts/'+old,'/scripts/'+new)
        text=re.sub(r'(\btype=[\"\'])'+re.escape(old)+r'([\"\'])',lambda m:m[1]+new+m[2],text)
        text=re.sub(r'(\bexecutable\s*=\s*[\"\'])'+re.escape(old)+r'([\"\'])',lambda m:m[1]+new+m[2],text)
    text=text.replace('_foscar_','_ctrl_')
    for i,url in enumerate(urls):text=text.replace('@@UPSTREAM_URL_'+str(i)+'@@',url)
    return text

def current_python(source,relative):
    """Rename import statements and references, retaining upstream comments/credits."""
    p=Path(relative)
    parts=p.parts
    parent='.'.join(parts[1:-1])
    new_parent=module_name(parent)
    forward_parts={k.rsplit('.',1)[-1]:v.rsplit('.',1)[-1] for k,v in MODULES.items()}
    lines=source.splitlines(keepends=True);offsets=[0]
    for line in lines:offsets.append(offsets[-1]+len(line))
    edits=[]
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node,(ast.Import,ast.ImportFrom)):continue
        before=ast.dump(node)
        if isinstance(node,ast.Import):
            for item in node.names:item.name=module_name(item.name)
        else:
            original_module=node.module
            if node.level:
                anchor=parent.split('.')[:len(parent.split('.'))-node.level+1]
                base='.'.join(anchor+([node.module] if node.module else []))
                new_anchor=new_parent.split('.')[:len(new_parent.split('.'))-node.level+1]
                node.module=module_name(base)[len('.'.join(new_anchor)):].lstrip('.') or None
            else:node.module=module_name(node.module or '')
            is_package=(original_module is None or original_module in PACKAGES or original_module in PACKAGES.values()
                        or original_module in ('states','foscar_planning.states','ctrl_planning.states'))
            if is_package:
                for item in node.names:
                    name=forward_parts.get(item.name,item.name)
                    if name!=item.name:
                        if item.asname is None:item.asname=item.name
                        item.name=name
        if ast.dump(node)!=before:
            edits.append((offsets[node.lineno-1]+node.col_offset,offsets[node.end_lineno-1]+node.end_col_offset,ast.unparse(node)))
    for start,end,value in sorted(edits,reverse=True):source=source[:start]+value+source[end:]
    source=ctrl_text(source)
    source=source.replace("f'ctrl_planning.states.{name}'","f'ctrl_planning.states.ctrl_{name}'")
    source=source.replace("f'ctrl_planning.states.{name}_'","f'ctrl_planning.states.ctrl_{name}_'")
    return source

def legacy_text(text):
    text=_replace(text,REVERSE_MODULES)
    text=text.replace('.states.ctrl_', '.states.')
    text=_replace(text,{v:k for k,v in FILES.items()})
    for old,new in SCRIPTS.items():
        text=text.replace('/scripts/'+new,'/scripts/'+old)
        text=re.sub(r'(\btype=[\"\'])'+re.escape(new)+r'([\"\'])',lambda m:m[1]+old+m[2],text)
        text=re.sub(r'(\bexecutable\s*=\s*[\"\'])'+re.escape(new)+r'([\"\'])',lambda m:m[1]+old+m[2],text)
    return text

class LegacyNames(ast.NodeTransformer):
    def visit_Import(self,node):
        for item in node.names:
            item.name=legacy_text(item.name)
            if item.asname==item.name: item.asname=None
        return node

    def visit_ImportFrom(self,node):
        if node.module:
            node.module=(legacy_text(node.module) if not node.level else
                         '.'.join(MODULE_PARTS.get(p,p) for p in node.module.split('.')))
        for item in node.names:
            item.name=MODULE_PARTS.get(item.name,item.name)
            if item.asname==item.name: item.asname=None
        return node

    def visit_Name(self,node):
        node.id=REVERSE_PACKAGES.get(node.id,node.id)
        return node

    def visit_Attribute(self,node):
        # Qualified module access is rare; relative imports retain old local aliases.
        parts=[];part=node
        while isinstance(part,ast.Attribute): parts.append(part.attr);part=part.value
        if isinstance(part,ast.Name):
            full='.'.join([part.id]+list(reversed(parts)))
            restored=legacy_text(full)
            if restored!=full:
                return ast.copy_location(ast.parse(restored,mode='eval').body,node)
        return self.generic_visit(node)

    def visit_Constant(self,node):
        if isinstance(node.value,str): node.value=legacy_text(node.value)
        return node

def legacy_ast(source):
    return LegacyNames().visit(ast.parse(source))

def canonical_python(source):
    return ast.unparse(legacy_ast(source))+'\n'
