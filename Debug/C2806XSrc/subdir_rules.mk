################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
C2806XSrc/%.obj: ../C2806XSrc/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --opt_for_speed=2 --fp_mode=strict --fp_reassoc=on --include_path="D:/44 F28069HMobis72VPackBMSR20/F28069PackBMS" --include_path="C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="D:/44 F28069HMobis72VPackBMSR20/F28069PackBMS/SysInclude" --include_path="D:/44 F28069HMobis72VPackBMSR20/00 C2806X/include" --advice:performance=all -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="C2806XSrc/$(basename $(<F)).d_raw" --obj_directory="C2806XSrc" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

C2806XSrc/%.obj: ../C2806XSrc/%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --opt_for_speed=2 --fp_mode=strict --fp_reassoc=on --include_path="D:/44 F28069HMobis72VPackBMSR20/F28069PackBMS" --include_path="C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="D:/44 F28069HMobis72VPackBMSR20/F28069PackBMS/SysInclude" --include_path="D:/44 F28069HMobis72VPackBMSR20/00 C2806X/include" --advice:performance=all -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="C2806XSrc/$(basename $(<F)).d_raw" --obj_directory="C2806XSrc" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


