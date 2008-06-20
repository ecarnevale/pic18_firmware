# Emanuel Carnevale 2008
#

require 'rake'
require 'rake/clean'
require 'fileutils'
include FileUtils

#wine /home/emanuel/.wine/drive_c/mcc18/bin/mcc18 -p=18F2520 "onewheel.c" -fo="onewheel.o"
#wine /home/emanuel/.wine/drive_c/mcc18/bin/mplink.exe 18f2520.lkr onewheel.o /o onewheel.out /l"/home/emanuel/.wine/drive_c/mcc18\lib"

CLEAN.include ["dist/","*.o", "*.cof", "*.err", "*.map", "*.hex", "*.mcp", "*.mcs", "*.mcw"]

compiler="compiler"
pic="PIC18"

desc 'Unsurprisingly, compile task compiles the .c files into objects'
task :compile do
  FileList["*.c"].each { |c_file|
    puts "#{compiler} -p=#{pic} #{c_file} -fo=#{c_file} -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-"
  }
  
end

desc 'links all the objects files into a hex file'
task :link => [:compile] do
  FileList["*.o"].each { |o_file|
    #wine /home/emanuel/.wine/drive_c/mcc18/bin/mplink.exe 18f2520.lkr onewheel.o /o onewheel.out /l"/home/emanuel/.wine/drive_c/mcc18\lib"
  }
end

#rule '.o' => ['.c'] do |t|
#  puts "#{compiler} -p=#{pic} #{t.source} -fo=#{t.name} -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-"
#end

