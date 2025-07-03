#pragma once

#include "kim2_comm.hpp"
#include "kineis_device.hpp"
#include "nrfx_uarte.h"


class KIM2Device : public KIM2CommEventListener, public KineisDevice {
private:
    KIM2Comm m_kim2_comm;

public:
    KIM2Device();
    ~KIM2Device();

private:
    bool m_cmd_is_ok;

    void detect_device(void);

	// Events
	void react(const KIM2CommEventOk&) override;
};