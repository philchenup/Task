#ifndef FLOWEDITOR_STEPREGISTRY_H
#define FLOWEDITOR_STEPREGISTRY_H

#include "Types.h"

#include <QHash>
#include <QStringList>

class StepRegistry
{
public:
    StepRegistry();

    const StepDesc *find(const QString &typeId) const;
    QStringList categories() const;
    QVector<StepDesc> stepsInCategory(const QString &category) const;

private:
    void addStep(const StepDesc &desc);

    QVector<StepDesc> m_steps;
    QHash<QString, int> m_index;
    QStringList m_categories;
};

#endif
