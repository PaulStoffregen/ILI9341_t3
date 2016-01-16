#! /usr/bin/perl

$name = $ARGV[$#ARGV];

if ($#ARGV == 1) {
  $param = $ARGV[0];
}

$name or die "Usage ./ttf_to_ili9341.pl [-digits] <name>\n";
-r "$name.ttf" or die "Can't read file $name.ttf\n";

if ($param == "-digits") {
  $fullname = $name . "_Digits";
}

system "rm font_$fullname.c\n";
open C, ">font_$fullname.c";
print C "#include \"font_$fullname.h\"\n\n";
close C;

foreach $size (8, 9, 10, 11, 12, 13, 14, 16, 18, 20, 24, 28, 32, 40, 48, 60, 72, 96) {
	print "converting size $size\n";
	system "otf2bdf -p $size $name.ttf | ./bdf_to_ili9341 $param >> font_$fullname.c\n";
}

print "writing header file\n";
open C, "grep ILI9341_t3_font_t font_$fullname.c |";
open H, ">font_$fullname.h";

print H "#ifndef _ILI9341_t3_font_${fullname}_\n";
print H "#define _ILI9341_t3_font_${fullname}_\n\n";
print H "#include \"ILI9341_t3.h\"\n\n";
print H "#ifdef __cplusplus\n";
print H "extern \"C\" {\n";
print H "#endif\n\n";

while (<C>) {
	chop;
	next unless /^const/;
	s/ = {$//;
	print H "extern $_;\n";
	#print "$_\n";
}
close C;

print H "\n";
print H "#ifdef __cplusplus\n";
print H "} // extern \"C\"\n";
print H "#endif\n\n";
print H "#endif\n";
close H;

