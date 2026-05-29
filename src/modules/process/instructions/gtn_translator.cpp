#include "modules/process/instructions/i_controller_translator.h"

#include <QObject>

namespace lcnc::process {

bool GtnControllerTranslator::translate(const ProcessCommandBuffer& buffer,
                                        QStringList* output,
                                        QString* errorMessage) const
{
    Q_UNUSED(errorMessage);
    if (!output)
        return true;

    output->clear();
    for (const ProcessCommand& command : buffer.commands()) {
        output->append(QObject::tr("GTN:%1:%2")
                           .arg(static_cast<int>(command.type))
                           .arg(command.description));
    }
    return true;
}

} // namespace lcnc::process