prefix=`dirname $0` ;

js="jquery.js bootstrap.js jquery.jsonrpcclient.js";
css="*.css"
html="*.html"

rm $prefix/web_include.h
rm $prefix/js.all
rm $prefix/css.all
rm $prefix/web.all

for x in $js;
do
  echo adding $prefix/$x to $prefix/js.all;
  cat $prefix/$x >> $prefix/js.all
done

for x in $css;
do
  echo adding $prefix/$x to $prefix/css.all;
  cat $prefix/$x >> $prefix/css.all
done

for x in $html;
do
  echo adding $prefix/$x to $prefix/web.all;
  cat $prefix/$x >> $prefix/web.all
done

cd $prefix;
xxd -i js.all  >> web_include.h;
xxd -i css.all >> web_include.h;
xxd -i web.all >> web_include.h;
cd $PWD;
