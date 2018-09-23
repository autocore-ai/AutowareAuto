#!/usr/bin/env python3

import os
import shutil
import subprocess

from lxml import etree

XMLTEMPLATE = '<script id="searchdata_xml" type="text/xmldata">{}</script>'

def build():
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(ws, __file__)
    builddir = os.path.join(ws, 'docs', 'build')
    if not os.path.exists(builddir):
        os.makedirs(builddir)
    subprocess.run(['doxygen', 'docs/.doxygen/Doxyfile'], cwd=ws)
    shutil.copyfile(
        os.path.join(ws, 'docs', '.doxygen', 'search.js'),
        os.path.join(ws, 'docs', 'build', 'html', 'search', 'search.js'))
    with open(os.path.join(ws, 'docs', 'build', 'html', 'search.html'), 'a') as html:
        with open(os.path.join(ws, 'docs', 'build', 'searchdata.xml'), 'rb') as xml:
            root = etree.fromstring(xml.read())
            pages = root.xpath('/add/doc/field[@name="type" and text() ="page"]/..')
            for page in pages:
                url = page.xpath('field[@name="url"]/text()')[0]
                with open(os.path.join(ws, 'docs', 'build', 'html', url)) as pagefile:
                    pageroot = etree.HTML(pagefile.read())
                    title = pageroot.xpath('//div[@class="title"]/text()')[0].strip()
                    field = etree.Element('field')
                    field.attrib['name'] = 'title'
                    field.text = title
                    page.append(field)
            html.write(XMLTEMPLATE.format(etree.tostring(root).decode('utf8')))

if __name__ == '__main__':
    build()

