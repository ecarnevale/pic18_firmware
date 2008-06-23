# Emanuel Carnevale 2008
#

require 'rake'
require 'rake/clean'
require 'fileutils'
include FileUtils

#wine /home/emanuel/.wine/drive_c/mcc18/bin/mcc18 -p=18F2520 "onewheel.c" -fo="onewheel.o"
#wine /home/emanuel/.wine/drive_c/mcc18/bin/mplink.exe 18f2520.lkr onewheel.o /o onewheel.out /l"/home/emanuel/.wine/drive_c/mcc18\lib"

CLEAN.include ["dist/","*.o", "*.cof", "*.err", "*.map", "*.hex", "*.mcp", "*.mcs", "*.mcw", "*.cod", "*.out", "*.lst"]

MCC_BIN = "wine /home/emanuel/.wine/drive_c/mcc18/bin/"
MCC_LIB = "/home/emanuel/.wine/drive_c/mcc18/bin/lib/"
COMPILER = "mcc18.exe"
LINKER = "mplink.exe"
PIC="18F2520"

desc 'Unsurprisingly, compile task compiles the .c files into objects'
task :compile do
  FileList["*.c"].each { |c_file|
    sh "#{MCC_BIN}#{COMPILER} -p=#{PIC} #{c_file} -fo=#{File.basename(c_file,'.c')+'.o'} -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-"
  }
  
end

desc 'builds the gyro_test version'
file 'gyro_test.hex' => FileList["*.o"] do |t|
  link t.name, t.prerequisites
end

desc 'links all the objects files into a hex file'
task :link => [:compile, 'gyro_test.hex']

def link(outfile, o_files)
  sh "#{MCC_BIN}#{LINKER} -p=#{PIC}.lkr #{o_files} /o #{outfile} /l\"#{MCC_LIB}\""
end

#rule '.o' => ['.c'] do |t|
#  puts "#{compiler} -p=#{pic} #{t.source} -fo=#{t.name} -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-"
#end

