#pragma once

#include "modules/cad/i_cad_facade.h"

class TaskProgress;

namespace lcnc::cad {

class ModelEnvelopeService final
{
public:
    static bool generate(const CadModelEnvelopeRequest& request,
                         TaskProgress* progress,
                         CadModelEnvelopeResult* result,
                         QString* errorMessage = nullptr);
};

} // namespace lcnc::cad
