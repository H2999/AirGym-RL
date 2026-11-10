#!/usr/bin/env node
// Read local CAD only. Requires the Apache-2.0 solidworks-converters package.
// The assembly XML retains the saved component transforms; no mates are solved.
'use strict';
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const args = process.argv.slice(2);
function arg(flag) {
  const i = args.indexOf(flag);
  if (i < 0 || !args[i + 1]) throw Error('Missing ' + flag);
  return path.resolve(args[i + 1]);
}
const readerRoot = arg('--reader');
const source = arg('--source');
const output = arg('--output');
const reader = require(path.join(readerRoot, 'src'));
const assembly = path.join(source, '无人之机.SLDASM');
const streams = reader.container.modern.decompressOpenSX(
  fs.readFileSync(assembly), reader.inflateRaw, reader.inflateZlib);
if (!streams['swXmlContents/COMPINSTANCETREE']) throw Error('Missing saved component tree');
fs.mkdirSync(output, {recursive: true});
fs.writeFileSync(path.join(output, 'assembly.xml'), streams['swXmlContents/COMPINSTANCETREE']);
if (streams.PreviewPNG) fs.writeFileSync(path.join(output, 'cad_preview.png'), streams.PreviewPNG);
const records = [];
const tree = streams['swXmlContents/COMPINSTANCETREE'].toString('utf8');
const referenced = new Set([...tree.matchAll(/swPath="([^"]+\.SLDPRT)"/gi)]
  .map(match => match[1].split('\\').pop()));
for (const filename of fs.readdirSync(source).filter(f => referenced.has(f)).sort()) {
  const input = path.join(source, filename);
  const parsed = reader.parse(input);
  const mesh = reader.meshModel(input);
  if (!mesh.triangleCount) throw Error(filename + ': empty mesh');
  // SolidWorks display coordinates are metres. Override the STL writer's mm default.
  const meshFile = 'part_' + records.length.toString().padStart(2, '0') + '.stl';
  fs.writeFileSync(path.join(output, meshFile), reader.toSTL(input, {scale: 1}));
  records.push({filename, mesh: meshFile, mesh_source: mesh.source, faces: parsed.faces.length,
    triangles: mesh.triangleCount, parser_warnings: (parsed.warnings || []).length,
    sha256: crypto.createHash('sha256').update(fs.readFileSync(input)).digest('hex')});
}
const report = {assembly: path.basename(assembly),
  assembly_sha256: crypto.createHash('sha256').update(fs.readFileSync(assembly)).digest('hex'),
  geometry_source: 'saved SolidWorks display tessellation; assembly saved XML transforms',
  coordinate_unit: 'metre', reader: 'https://github.com/blussyya/solidworks-converters', parts: records};
fs.writeFileSync(path.join(output, 'parts.json'), JSON.stringify(report, null, 2) + '\n');
console.log('Extracted ' + records.length + ' part meshes and saved assembly transforms to ' + output);
