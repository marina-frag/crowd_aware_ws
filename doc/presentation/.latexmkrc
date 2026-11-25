# xelatex: the slides use fontspec + polyglossia (pdflatex cannot compile them)
$pdf_mode = 5;
$xelatex = "xelatex -interaction=nonstopmode -synctex=1 %O %S";
