// Drop-in replacement for doxygen's search.js with improved fulltext search
// Based on Doxygen 1.8.11 search.js
// Modified for AutowareAuto
var searchResultsText=["Sorry, no documents matching your query.","Found <b>1</b> document matching your query.","Found <b>$num</b> documents matching your query. Showing best matches first."];
var serverUrl="";
var tagMap = {
};

function SearchBox(name, resultsPath, inFrame, label)
{
  this.searchLabel = label;
  this.DOMSearchField = function()
  {  return document.getElementById("MSearchField");  }
  this.DOMSearchBox = function()
  {  return document.getElementById("MSearchBox");  }
  this.OnSearchFieldFocus = function(isActive)
  {
    if (isActive)
    {
      this.DOMSearchBox().className = 'MSearchBoxActive';
      var searchField = this.DOMSearchField();
      if (searchField.value == this.searchLabel)
      {
        searchField.value = '';
      }
    }
    else
    {
      this.DOMSearchBox().className = 'MSearchBoxInactive';
      this.DOMSearchField().value   = this.searchLabel;
    }
  }
}

function trim(s) {
  return s?s.replace(/^\s\s*/, '').replace(/\s\s*$/, ''):'';
}

function getURLParameter(name) {
  return decodeURIComponent((new RegExp('[?|&]'+name+
         '='+'([^&;]+?)(&|#|;|$)').exec(location.search)
         ||[,""])[1].replace(/\+/g, '%20'))||null;
}

var entityMap = {
  "&": "&amp;",
  "<": "&lt;",
  ">": "&gt;",
  '"': '&quot;',
  "'": '&#39;',
  "/": '&#x2F;'
};

function escapeHtml(s) {
  return String(s).replace(/[&<>"'\/]/g, function (s) {
    return entityMap[s];
  });
}

async function searchFor(query,page,count) {
  function showData(data) {
    var results = $('#searchresults');
    $('#MSearchField').val(query);
    if (data.hits>0) {
      if (data.hits==1) {
        results.html('<p>'+searchResultsText[1]+'</p>');
      } else {
        results.html('<p>'+searchResultsText[2].replace(/\$num/,data.hits)+'</p>');
      }

      var r='<table>';
      r += '<tr><td class="search-section" colspan=2>Documentation titles matching the search query</td></tr>';
      data.titleitems.forEach((item, index) => {
        r += `
          <tr class="searchresult">
            <td align="right">${index+1}.</td>
            <td>
                <a href="${escapeHtml(item.url)}">${escapeHtml(item.title)}</a>
            </td>
          </tr>
        `;
      });
      r += '<tr><td class="search-section" colspan=2>Content matching the search query</td></tr>';
      $.each(data.items, function(i,item){
        var prefix = tagMap[item.tag];
        const type = item.type !== 'page' ? item.type : '';
        if (prefix) prefix+='/'; else prefix='';
        r+='<tr class="searchresult">'+
           '<td align="right">'+(data.first+i+1)+'.</td>'+
           '<td>'+escapeHtml(type)+'&#160;'+
                '<a href="'+escapeHtml(prefix+item.url)+
                '">'+escapeHtml(item.title || item.name)+'</a>';
        if (item.type=="source") {
          var l=item.url.match(/[1-9][0-9]*$/);
          if (l) r+=' at line '+parseInt(l[0]);
        }
        r+='</td>';
        for (var i=0;i<item.fragments.length;i++)
        {
          r+='<tr><td></td><td>'+item.fragments[i]+'</td></tr>';
        }
        r+='</tr>';
      });
      r+='</table>';
      if (data.pages>1) // write multi page navigation bar
      {
        r+='<div class="searchpages">';
        if (data.page>0)
        {
          r+='<span class="pages"><a href="javascript:searchFor(\''+escapeHtml(query)+'\','+(page-1).toString()+','+count.toString()+')">&laquo;</a></span>&nbsp;';
        }
        var firstPage = data.page-5;
        var lastPage  = data.page+5;
        if (firstPage<0)
        {
          lastPage-=firstPage;
          firstPage=0;
        }
        if (lastPage>data.pages)
        {
          lastPage=data.pages;
        }
        for(var i=firstPage;i<lastPage;i++)
        {
          if (i==data.page)
          {
            r+='<span class="pages"><b>'+(i+1).toString()+'</b></span>&nbsp;';
          }
          else
          {
            r+='<span class="pages"><a href="javascript:searchFor(\''+escapeHtml(query)+'\','+i.toString()+','+count.toString()+')">'+(i+1).toString()+'</a></span>&nbsp;';
          }
        }
        if (data.page+1<data.pages)
        {
          r+='<span class="pages"><a href="javascript:searchFor(\''+escapeHtml(query)+'\','+(page+1).toString()+','+count.toString()+')">&raquo;</a></span>';
        }
        r+='</div>';
      }
      results.append(r);
    } else {
      results.html('<p>'+searchResultsText[0]+'</p>');
    }
  }

  const data = {
    'hits': 0,
    'first': 0,
    'count': 0,
    'page': 1,
    'pages': 1,
    'query': 'list',
    'items':[],
    'titleitems':[],
  };

  const rxq = new RegExp(query.replace(/[-\/\\^$*+?.()|[\]{}]/g, '\\$&'), 'gi');
  const text = document.getElementById('searchdata_xml').innerHTML.trim();
  const docs = new window.DOMParser().parseFromString(text, 'application/xml').childNodes[0].childNodes;
  const ctxlen = 40;

  function format(strings, ...values) {
    return String.raw(
        {raw: strings.map(s => s.replace(/\n          $/, ''))},
        ...values
    );
  }

  docs.forEach((doc, i) => {
    if (doc.nodeType === 3) {
      return;
    }
    const item = {};
    Array.from(doc.children).forEach(field => {
      const key = field.getAttribute('name');
      const value = field.textContent;
      item[key] = value;
    });

    if (item.title && item.title.match(rxq)) {
      data.count++;
      data.hits++;
      data.titleitems.push(item);
    } else if (item.text && item.text.match(rxq)) {
      data.count++;
      data.hits++;
      data.items.push(item);
      item.fragments = [];

      let res;
      const len = item.text.length;
      while ((res = rxq.exec(item.text))) {
        const start = Math.max(res.index - ctxlen, 0);
        const stop = Math.min(res.index + query.length + ctxlen, len);
        item.fragments.push(format`
          ${start ? '...' : ''}
          ${item.text.substring(start, res.index)}
          <span class="hl">${item.text.substring(res.index, res.index + query.length)}</span>
          ${item.text.substring(res.index + query.length, stop)}${stop < len ? '...' : ''}
        `);
        if (item.fragments.length === 3) {
          break;
        }
      }
    }
  });
  showData(data);
}

$(document).ready(function() {
  var query = trim(getURLParameter('query'));
  if (query) {
    searchFor(query,0,20);
  } else {
    var results = $('#results');
    results.html('<p>Sorry, no documents matching your query.</p>');
  }
});
