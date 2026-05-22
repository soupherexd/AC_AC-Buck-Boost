################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
F2806x_headers/source/%.obj: ../F2806x_headers/source/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --vcu_support=vcu0 --include_path="C:/Users/Star.Dust/Desktop/project/F2806x_common/include" --include_path="C:/Users/souph/Documents/CCSProjects/ACAC/AC_AC Buck-Boost/F2806x_common/include" --include_path="C:/Users/souph/Documents/CCSProjects/ACAC/AC_AC Buck-Boost/F2806x_headers/include" --include_path="C:/Users/Souphere/Desktop/project/project/F2806x_common/include" --include_path="C:/Users/Souphere/Desktop/project/project/F2806x_headers/include" --include_path="C:/Users/Star.Dust/Desktop/project/F2806x_headers/include" --include_path="D:/work/CCS/TMS320F28069/F2806x_headers/include" --include_path="C:/Users/Star.Dust/Desktop/project1/F2806x_headers/include" --include_path="C:/Users/Star.Dust/Desktop/project1/F2806x_common/include" --include_path="D:/work/CCS/TMS320F28069/F2806x_common/include" --include_path="C:/Users/souph/Documents/CCSProjects/ACAC/AC_AC Buck-Boost" --include_path="C:/ti/ccs1250/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="F2806x_headers/source/$(basename $(<F)).d_raw" --obj_directory="F2806x_headers/source" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


