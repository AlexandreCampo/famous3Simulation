

solution "experiment"
   configurations { "debug", "release" , "profile" }

   --- ============================= LINUX ==================================
   if os.is ("linux") then

      includedirs { "/usr/include/libfamous" }
      includedirs { "/usr/local/include/libfamous" }
      includedirs { "/usr/include/bullet" }
      includedirs { "/usr/include/eigen3" }

      libdirs { os.findlib("glut"), os.findlib("GL"), os.findlib("GLU"), 
      	        os.findlib("gsl"), os.findlib("BulletDynamics"), 
                os.findlib("boost_program_options"), os.findlib("tinyxml"),
		os.findlib("famous")}
      
      links { "famous", "BulletDynamics", "BulletCollision", "LinearMath", 
              "gsl", "gslcblas", "glut", "GL", "GLU", "boost_program_options", 
              "tinyxml", "osg", "osgGA", "osgDB", "osgViewer", "osgText"}


   --- ============================= MACOSX =================================
    elseif os.is ("macosx") then

      includedirs { "/opt/local/include/libfamous" }
      includedirs { "/opt/local/include/bullet" }
      includedirs { "/opt/local/include" }

      libdirs { os.findlib("glut"), os.findlib("GL"), os.findlib("GLU"), 
                os.findlib("gsl"), os.findlib("BulletDynamics"), 
                os.findlib("boost_program_options"), os.findlib("tinyxml")}

      libdirs { "/opt/local/lib" }

      links { "famous", "BulletDynamics", "BulletCollision", "LinearMath", 
              "gsl", "gslcblas", "glut", "GL", "GLU", "boost_program_options-mt", "tinyxml"}

   end


   --- ============================= GENERIC =================================

   project "experiment"
      kind "ConsoleApp"
      language "C++"
      files { "**.h", "**.cpp" }

      configuration "release"
         buildoptions {"-std=c++11"}
         defines { "NDEBUG" }
         flags { "OptimizeSpeed", "EnableSSE", "EnableSSE2", "FloatFast", "NoFramePointer"}    

      configuration "debug"
         buildoptions {"-std=c++11"}
         defines { "DEBUG" }
         flags { "Symbols" }
 
      configuration "profile"
         buildoptions {"-std=c++11"}
         defines { "DEBUG" }
         flags { "Symbols" }
	 buildoptions { "-pg" }	
	 linkoptions { "-pg" }	
