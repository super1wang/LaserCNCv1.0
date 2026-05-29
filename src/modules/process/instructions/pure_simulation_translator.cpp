#include "modules/process/instructions/i_controller_translator.h"

#include <QObject>

namespace lcnc::process {

bool PureSimulationTranslator::translate(const ProcessCommandBuffer& buffer,
                                         QStringList* output,
                                         QString* errorMessage) const
{
    Q_UNUSED(errorMessage);
    if (!output)
        return true;

    output->clear();
    for (const ProcessCommand& command : buffer.commands()) {
        if (!command.description.isEmpty()) {
            output->append(command.description);
            continue;
        }
        output->append(QObject::tr("Process command %1").arg(static_cast<int>(command.type)));
    }
    return true;
}

} // namespace lcnc::process