#!/usr/bin/env python3

'''Build documentation with doxygen and inject customized search code'''

import os
import shutil
import subprocess
import sys

from lxml import etree

XMLTEMPLATE = '<script id="searchdata_xml" type="text/xmldata">{}</script>'


def build():
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print(ws, __file__)
    builddir = os.path.join(ws, 'docs', '_build')
    if not os.path.exists(builddir):
        os.makedirs(builddir)
    try:
        on_rtd = os.environ.get('READTHEDOCS', None) == 'True'
        if on_rtd:
            docs_path = '{}/docs/.doxygen'.format(ws)
            doxygen_base = 'doxygen-1.8.17-bundle'
            doxygen_tarball = '{}.tar.gz'.format(doxygen_base)
            doxygen_binary_path = '{}/doxygen-1.8.17'.format(doxygen_base)
            url = 'https://autoware-auto.s3.us-east-2.amazonaws.com/{}'.format(doxygen_tarball)
            download_cmd = 'wget {}'.format(url)
            file_cmd = 'file {}'.format(doxygen_tarball)
            tar_cmd = 'tar xvf {}'.format(doxygen_tarball)
            chmod_cmd = 'chmod -v a+x {}'.format(doxygen_base)
            doxygen_cmd = 'LD_LIBRARY_PATH={} {} docs/.doxygen/Doxyfile'.format(
                doxygen_base, doxygen_binary_path)
            subprocess.run(download_cmd, shell=True, cwd=ws, check=True)
            subprocess.run(file_cmd, shell=True, cwd=ws, check=True)
            subprocess.run(tar_cmd, shell=True, cwd=ws, check=True)
            subprocess.run(chmod_cmd, shell=True, cwd=ws, check=True)
            subprocess.run(doxygen_cmd, shell=True, cwd=ws, check=True)
        else:
            subprocess.run(['doxygen', 'docs/.doxygen/Doxyfile'], cwd=ws, check=True)
    except subprocess.CalledProcessError as err:
        print('\nDoxygen failed to generate documentation: {}\n'.format(err))
        return 1
    shutil.copyfile(
        os.path.join(ws, 'docs', '.doxygen', 'search.js'),
        os.path.join(builddir, 'html', 'search', 'search.js'))
    with open(os.path.join(builddir, 'html', 'search.html'), 'a') as html:
        with open(os.path.join(builddir, 'searchdata.xml'), 'rb') as xml:
            root = etree.fromstring(xml.read())
            pages = root.xpath('/add/doc/field[@name="type" and text() ="page"]/..')
            for page in pages:
                url = page.xpath('field[@name="url"]/text()')[0]
                with open(os.path.join(builddir, 'html', url)) as pagefile:
                    pageroot = etree.HTML(pagefile.read())
                    title = pageroot.xpath('//div[@class="title"]/text()')[0].strip()
                    field = etree.Element('field')
                    field.attrib['name'] = 'title'
                    field.text = title
                    page.append(field)
            html.write(XMLTEMPLATE.format(etree.tostring(root).decode('utf8')))
    print('\nDocumentation has been built in: {}\n'.format(os.path.join(builddir, 'html', 'index.html')))
    return 0


if __name__ == '__main__':
    sys.exit(build())
