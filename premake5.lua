-- Irlan Robson
-- Bounce Softbody premake script
-- http://industriousone.com/premake

-- variable paths for the case files are moved
solution_name = "bounce_softbody"
working_dir = "."
solution_dir = "build/"
external_dir = "external/"
bounce_softbody_inc_dir = "include/"
bounce_softbody_src_dir = "src/"
tests_inc_dir = "tests/"
tests_src_dir = "tests/"
obj_dir = "/obj/"
bin_dir = "/bin/"

-- or "" to make --help work
action = _ACTION or ""

-- premake main
workspace(solution_name)
	configurations { "debug", "release" }
	location(solution_dir .. "/" .. action)
	symbols "On"
	warnings 'Extra'

	filter "system:windows" 
		platforms { "x86", "x86_64" }
	    	defaultplatform "x86_64"
		defines { "_CRT_SECURE_NO_WARNINGS", "_WIN32", "WIN32", "_WINDOWS", "TRILIBRARY" }
		
	filter "system:linux" 
		platforms { "x86", "x86_64" }
		defaultplatform "x86_64"
		cppdialect "C++11"
	
	filter {}
	
	filter "configurations:debug"
		defines { "DEBUG" }
		optimize "Off"
		targetdir ( solution_dir .. action .. bin_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		objdir ( "!" .. solution_dir .. action .. obj_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )	
    
	filter "configurations:release"
		defines { "NDEBUG" }
		optimize "On"
		targetdir ( solution_dir .. action .. bin_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		objdir ( "!" .. solution_dir .. action .. obj_dir .. "%{cfg.platform}/%{cfg.buildcfg}/%{prj.name}" )
		
	filter {}
	
	project "bounce_softbody"
		kind "StaticLib"
		location ( solution_dir .. action )
		includedirs { bounce_softbody_inc_dir }
		
		files 
		{ 
			bounce_softbody_inc_dir .. "/bounce_softbody/**.h", 
			bounce_softbody_inc_dir .. "/bounce_softbody/**.inl",
			bounce_softbody_src_dir .. "/bounce_softbody/**.cpp" 
		}
			
	project "glad"
		kind "StaticLib"
		language "C"
		location ( solution_dir .. action )
		includedirs { external_dir .. "/glad/include" }
		
		files 
		{ 
			external_dir .. "/glad/include/KHR/khrplatform.h",
			external_dir .. "/glad/include/glad.h", 
			external_dir .. "/glad/src/glad.c",
		}
		
		filter { "system:linux" } 
			files 
			{ 
				external_dir .. "/glad/include/glad_glx.h", 
				external_dir .. "/glad/src/glad_glx.c",
			}
			
	project "glfw"
		kind "StaticLib"
		language "C"
		location ( solution_dir .. action )
		includedirs { external_dir .. "/glfw/include" }
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.c" }

		files
		{
			external_dir .. "/glfw/include/glfw/glfw3.h",
			external_dir .. "/glfw/include/glfw/glfw3native.h",
			
			external_dir .. "/glfw/src/glfw_config.h",
			
			external_dir .. "/glfw/src/context.c", 		
			external_dir .. "/glfw/src/init.c",
			external_dir .. "/glfw/src/input.c",
			external_dir .. "/glfw/src/monitor.c",
			external_dir .. "/glfw/src/vulkan.c",
			external_dir .. "/glfw/src/window.c",
		}	
		
		filter "system:windows" 
			files 
			{	 
				external_dir .. "/glfw/src/win32_platform.h",
				external_dir .. "/glfw/src/win32_joystick.h",
				external_dir .. "/glfw/src/wgl_context.h",
				external_dir .. "/glfw/src/egl_context.h",
				external_dir .. "/glfw/src/osmesa_context.h",
				
				external_dir .. "/glfw/src/win32_init.c",
				external_dir .. "/glfw/src/win32_joystick.c",
				external_dir .. "/glfw/src/win32_monitor.c",
				external_dir .. "/glfw/src/win32_time.c",
				external_dir .. "/glfw/src/win32_thread.c",
				external_dir .. "/glfw/src/win32_window.c",
				external_dir .. "/glfw/src/wgl_context.c",
				external_dir .. "/glfw/src/egl_context.c",
				external_dir .. "/glfw/src/osmesa_context.c",				
			}

		filter "system:linux" 
         	buildoptions { "-pthread" }
			files 
			{	 
				external_dir .. "/glfw/src/x11_platform.h",
				external_dir .. "/glfw/src/xkb_unicode.h",
				external_dir .. "/glfw/src/posix_time.h",	
				external_dir .. "/glfw/src/posix_thread.h",	
				external_dir .. "/glfw/src/glx_context.h",
				external_dir .. "/glfw/src/egl_context.h",
				external_dir .. "/glfw/src/osmesa_context.h",
				external_dir .. "/glfw/src/linux_joystick.h",
				
				external_dir .. "/glfw/src/x11_init.c",	
				external_dir .. "/glfw/src/x11_monitor.c",
				external_dir .. "/glfw/src/x11_window.c",
				external_dir .. "/glfw/src/xkb_unicode.c",
				external_dir .. "/glfw/src/posix_time.c",
				external_dir .. "/glfw/src/posix_thread.c",
				external_dir .. "/glfw/src/glx_context.c",
				external_dir .. "/glfw/src/egl_context.c",					
				external_dir .. "/glfw/src/osmesa_context.c",
				external_dir .. "/glfw/src/linux_joystick.c",				
			}	
			
	project "imgui"
		kind "StaticLib"
		language "C++"
		location ( solution_dir .. action )
		includedirs { external_dir } 
		vpaths { ["Headers"] = "**.h", ["Sources"] = "**.cpp" }		
	
		files 
		{ 
			external_dir .. "/imgui/**.h", 
			external_dir .. "/imgui/**.cpp",			
		}
		
	project "testbed"
		kind "ConsoleApp"
		language "C++"
		location ( solution_dir .. action )
		includedirs 
		{ 
				external_dir, 
				external_dir .. "/glfw/include", 
				external_dir .. "/glad/include", 
				working_dir .. "/testbed", 
				bounce_softbody_inc_dir
		}
		
		files 
		{ 
			"testbed/**.h",
			"testbed/**.c",
			"testbed/**.cpp",
		}
		
		filter "system:windows" 
			links { "opengl32", "winmm" }
			
		filter "system:linux" 
			links { "GL", "X11", "Xrandr", "Xinerama", "Xcursor", "pthread", "dl" }
			linkoptions { "-no-pie" }
		
		filter {}
		
		links { "glad", "glfw", "imgui", "bounce_softbody" }
