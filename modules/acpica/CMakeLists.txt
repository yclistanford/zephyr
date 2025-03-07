# Copyright (c) 2023 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

set(ACPI_DIR ${ZEPHYR_CURRENT_MODULE_DIR}/)
set(INC_DIR ${ACPI_DIR}/source/include/)
set(SRC_DIR ${ACPI_DIR}/source)
set(COMP_DIR ${ACPI_DIR}/source/components)
set(PARENT_SRC_DIR ${ACPI_DIR}../../zephyr)
set(ACPI_PARENT_DIR ${ACPI_DIR}/../)

zephyr_include_directories(
	${PARENT_SRC_DIR}/include/
	${ACPI_PARENT_DIR}/
	${INC_DIR}/
	${INC_DIR}/platform/
	${SRC_DIR}/compiler/
	${ZEPHYR_CURRENT_MODULE_DIR}/generate/zephyr/
	${SRC_DIR}/tools/acpiexec/
	${SRC_DIR}/tools/acpidump/
)

if (CONFIG_ACPI)
	zephyr_library()

	add_compile_definitions(__ZEPHYR__)
	add_compile_definitions(ACPI_DEBUG_OUTPUT)
	add_compile_definitions(ACPI_EXAMPLE_APP)
	add_compile_definitions(CONFIG_EXTERNAL_LIBC)

	get_filename_component(libname "${SRC_DIR}/common/" NAME)

if (CONFIG_ACPI_DSDT_SUPPORT)
	zephyr_library_sources(
		${COMP_DIR}/dispatcher/dsargs.c
		${COMP_DIR}/dispatcher/dscontrol.c
		${COMP_DIR}/dispatcher/dsdebug.c
		${COMP_DIR}/dispatcher/dsfield.c
		${COMP_DIR}/dispatcher/dsinit.c
		${COMP_DIR}/dispatcher/dsmethod.c
		${COMP_DIR}/dispatcher/dsmthdat.c
		${COMP_DIR}/dispatcher/dsobject.c
		${COMP_DIR}/dispatcher/dsopcode.c
		${COMP_DIR}/dispatcher/dspkginit.c
		${COMP_DIR}/dispatcher/dsutils.c
		${COMP_DIR}/dispatcher/dswexec.c
		${COMP_DIR}/dispatcher/dswload.c
		${COMP_DIR}/dispatcher/dswload2.c
		${COMP_DIR}/dispatcher/dswscope.c
		${COMP_DIR}/dispatcher/dswstate.c
		${COMP_DIR}/events/evhandler.c
		${COMP_DIR}/events/evmisc.c
		${COMP_DIR}/events/evregion.c
		${COMP_DIR}/events/evrgnini.c
		${COMP_DIR}/events/evxface.c
		${COMP_DIR}/events/evxfregn.c
		${COMP_DIR}/executer/exconcat.c
		${COMP_DIR}/executer/exconfig.c
		${COMP_DIR}/executer/exconvrt.c
		${COMP_DIR}/executer/excreate.c
		${COMP_DIR}/executer/exdebug.c
		${COMP_DIR}/executer/exdump.c
		${COMP_DIR}/executer/exfield.c
		${COMP_DIR}/executer/exfldio.c
		${COMP_DIR}/executer/exmisc.c
		${COMP_DIR}/executer/exmutex.c
		${COMP_DIR}/executer/exnames.c
		${COMP_DIR}/executer/exoparg1.c
		${COMP_DIR}/executer/exoparg2.c
		${COMP_DIR}/executer/exoparg3.c
		${COMP_DIR}/executer/exoparg6.c
		${COMP_DIR}/executer/exprep.c
		${COMP_DIR}/executer/exregion.c
		${COMP_DIR}/executer/exresnte.c
		${COMP_DIR}/executer/exresolv.c
		${COMP_DIR}/executer/exresop.c
		${COMP_DIR}/executer/exserial.c
		${COMP_DIR}/executer/exstore.c
		${COMP_DIR}/executer/exstoren.c
		${COMP_DIR}/executer/exstorob.c
		${COMP_DIR}/executer/exsystem.c
		${COMP_DIR}/executer/extrace.c
		${COMP_DIR}/executer/exutils.c
		${COMP_DIR}/hardware/hwpci.c
		${COMP_DIR}/namespace/nsaccess.c
		${COMP_DIR}/namespace/nsalloc.c
		${COMP_DIR}/namespace/nsarguments.c
		${COMP_DIR}/namespace/nsconvert.c
		${COMP_DIR}/namespace/nsdump.c
		${COMP_DIR}/namespace/nseval.c
		${COMP_DIR}/namespace/nsinit.c
		${COMP_DIR}/namespace/nsload.c
		${COMP_DIR}/namespace/nsnames.c
		${COMP_DIR}/namespace/nsobject.c
		${COMP_DIR}/namespace/nsparse.c
		${COMP_DIR}/namespace/nspredef.c
		${COMP_DIR}/namespace/nsprepkg.c
		${COMP_DIR}/namespace/nsrepair.c
		${COMP_DIR}/namespace/nsrepair2.c
		${COMP_DIR}/namespace/nssearch.c
		${COMP_DIR}/namespace/nsutils.c
		${COMP_DIR}/namespace/nswalk.c
		${COMP_DIR}/namespace/nsxfeval.c
		${COMP_DIR}/namespace/nsxfname.c
		${COMP_DIR}/namespace/nsxfobj.c
		${COMP_DIR}/parser/psargs.c
		${COMP_DIR}/parser/psloop.c
		${COMP_DIR}/parser/psobject.c
		${COMP_DIR}/parser/psopcode.c
		${COMP_DIR}/parser/psopinfo.c
		${COMP_DIR}/parser/psparse.c
		${COMP_DIR}/parser/psscope.c
		${COMP_DIR}/parser/pstree.c
		${COMP_DIR}/parser/psutils.c
		${COMP_DIR}/parser/pswalk.c
		${COMP_DIR}/parser/psxface.c
		${COMP_DIR}/resources/rsxface.c
		${COMP_DIR}/resources/rsutils.c
		${COMP_DIR}/resources/rsaddr.c
		${COMP_DIR}/resources/rscalc.c
		${COMP_DIR}/resources/rscreate.c
		${COMP_DIR}/resources/rsdumpinfo.c
		${COMP_DIR}/resources/rsinfo.c
		${COMP_DIR}/resources/rsio.c
		${COMP_DIR}/resources/rsirq.c
		${COMP_DIR}/resources/rslist.c
		${COMP_DIR}/resources/rsmemory.c
		${COMP_DIR}/resources/rsmisc.c
		${COMP_DIR}/resources/rsserial.c
	)
endif (CONFIG_ACPI_DSDT_SUPPORT)
	zephyr_library_sources(
		${COMP_DIR}/tables/tbdata.c
		${COMP_DIR}/tables/tbfadt.c
		${COMP_DIR}/tables/tbfind.c
		${COMP_DIR}/tables/tbinstal.c
		${COMP_DIR}/tables/tbprint.c
		${COMP_DIR}/tables/tbutils.c
		${COMP_DIR}/tables/tbxface.c
		${COMP_DIR}/tables/tbxfload.c
		${COMP_DIR}/tables/tbxfroot.c
		${COMP_DIR}/utilities/utaddress.c
		${COMP_DIR}/utilities/utalloc.c
		${COMP_DIR}/utilities/utascii.c
		${COMP_DIR}/utilities/utbuffer.c
		${COMP_DIR}/utilities/utcache.c
		${COMP_DIR}/utilities/utcksum.c
		${COMP_DIR}/utilities/utcopy.c
		${COMP_DIR}/utilities/utdebug.c
		${COMP_DIR}/utilities/utdecode.c
		${COMP_DIR}/utilities/utdelete.c
		${COMP_DIR}/utilities/uterror.c
		${COMP_DIR}/utilities/uteval.c
		${COMP_DIR}/utilities/utexcep.c
		${COMP_DIR}/utilities/utglobal.c
		${COMP_DIR}/utilities/uthex.c
		${COMP_DIR}/utilities/utids.c
		${COMP_DIR}/utilities/utinit.c
		${COMP_DIR}/utilities/utlock.c
		${COMP_DIR}/utilities/utmath.c
		${COMP_DIR}/utilities/utmisc.c
		${COMP_DIR}/utilities/utmutex.c
		${COMP_DIR}/utilities/utobject.c
		${COMP_DIR}/utilities/utosi.c
		${COMP_DIR}/utilities/utownerid.c
		${COMP_DIR}/utilities/utnonansi.c
		${COMP_DIR}/utilities/utpredef.c
		${COMP_DIR}/utilities/utresrc.c
		${COMP_DIR}/utilities/utstate.c
		${COMP_DIR}/utilities/utstring.c
		${COMP_DIR}/utilities/utstrsuppt.c
		${COMP_DIR}/utilities/utstrtoul64.c
		${COMP_DIR}/utilities/utxface.c
		${COMP_DIR}/utilities/utxferror.c
		${COMP_DIR}/utilities/utxfinit.c
		${COMP_DIR}/utilities/utresdecode.c
		${COMP_DIR}/hardware/hwvalid.c
		${SRC_DIR}/os_specific/service_layers/oszephyr.c
	)
endif (CONFIG_ACPI)
