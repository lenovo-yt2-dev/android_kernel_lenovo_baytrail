#ifndef INTEL_DUAL_DISPLAY_H
#define INTEL_DUAL_DISPLAY_H
#include <drm/drmP.h> 

#ifdef CONFIG_DLP
/*2014-06-05 intel require don't change the order*/
enum dual_display_status_type {
    PIPE_INIT = 109,
    PIPE_OFF,
    PIPE_ON,
    PIPE_ON_PROCESSING,
    PIPE_OFF_PROCESSING
};
struct intel_dual_display_status{	
	enum dual_display_status_type pipea_status;
	enum dual_display_status_type pipeb_status;
	struct mutex dual_display_mutex;	/**< For others */
	bool (*call_back)(int);
};

#endif

#endif
