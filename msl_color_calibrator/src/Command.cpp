#include "Command.h"

#include <cstdlib>
#include <list>
#include <math.h>
#include <vector>

#include "EM.h"

#include "filters/FilterYUVExtractSubImages.h"

using namespace em;

Change::Change(int nPosition, unsigned char nOldValue, unsigned char nNewValue) {
	position = nPosition;
	oldValue = nOldValue;
	newValue = nNewValue;
}

AbstractCommand::~AbstractCommand() {
	for (list<Change*>::iterator it = m_changes.begin(); it != m_changes.end(); it++) {
		delete *it;
	}
}

void AbstractCommand::undo() {
	for (list<Change*>::iterator it = m_changes.begin(); it != m_changes.end(); it++) {
		Change *change = *it;
		m_lookupTable[change->position] = change->oldValue;
	}
}

void AbstractCommand::redo() {
	for (list<Change*>::iterator it = m_changes.begin(); it != m_changes.end(); it++) {
		Change *change = *it;
		m_lookupTable[change->position] = change->newValue;
	}
}

CommandList::~CommandList() {
	clear();
}

void CommandList::addCommand(AbstractCommand* command) {
	m_commands.push(command);
	while (!m_redoMemory.empty()) {
		AbstractCommand* command = m_redoMemory.top();
		m_redoMemory.pop();
		delete command;
	}
	emit listChanged();
}

bool CommandList::canUndo() {
	return !m_commands.empty();
}

bool CommandList::canRedo() {
	return !m_redoMemory.empty();
}

void CommandList::undo() {
	if (!m_commands.empty()) {
		AbstractCommand* command = m_commands.top();
		m_commands.pop();
		command->undo();
		m_redoMemory.push(command);
		emit listChanged();
	}
}

void CommandList::redo() {
	if (!m_redoMemory.empty()) {
		AbstractCommand* command = m_redoMemory.top();
		m_redoMemory.pop();
		command->redo();
		m_commands.push(command);
		emit listChanged();
	}
}

void CommandList::clear() {
	bool changed = false;

	while (!m_commands.empty()) {
		AbstractCommand* command = m_commands.top();
		m_commands.pop();
		delete command;
		changed = true;
	}

	while (!m_redoMemory.empty()) {
		AbstractCommand* command = m_redoMemory.top();
		m_redoMemory.pop();
		delete command;
		changed = true;
	}

	if (changed) {
		emit listChanged();
	}
}

CommandClearLookuptable::CommandClearLookuptable(unsigned char *lookupTable, bool defaultValues) {
	m_lookupTable = lookupTable;
	m_defaultValues = defaultValues;
}

bool CommandClearLookuptable::doIt() {
	unsigned char oldValue, newValue;

	bool changed = false;
	FilterYUVExtractSubImages filterYUVExtractSubImages(0, 0, 0);
	for(int u=0; u<256; u++) {
		for(int v=0; v<256; v++) {
			unsigned char defaultValue = filterYUVExtractSubImages.getLookupTableValue(u*256 + v) > 0 ? 255 : 0;

			oldValue = m_lookupTable[u*256 + v];
			newValue = (m_defaultValues ? defaultValue : 0);
			if (oldValue != newValue) {
				changed = true;
				m_lookupTable[u*256 + v] = newValue;
				m_changes.push_back(new Change(u*256 + v, oldValue, newValue));
			}
		}
	}

	// LOWER THRESHOLD
	oldValue = m_lookupTable[256 * 256 + 0];
	newValue = 0;
	if (oldValue != newValue) {
		changed = true;
		m_lookupTable[256 * 256 + 0] = newValue;
		m_changes.push_back(new Change(256 * 256 + 0, oldValue, newValue));
	}

	// UPPER THRESHOLD
	oldValue = m_lookupTable[256 * 256 + 1];
	newValue = 255;
	if (oldValue != newValue) {
		changed = true;
		m_lookupTable[256 * 256 + 1] = newValue;
		m_changes.push_back(new Change(256 * 256 + 1, oldValue, newValue));
	}

	return changed;
}

CommandYUV2Lookuptable::CommandYUV2Lookuptable(unsigned char * lookupTable, QPolygon poly, bool add) {
	m_lookupTable = lookupTable;
	m_poly = poly;
	m_add = add;
}

bool CommandYUV2Lookuptable::doIt() {
	bool changed = false;

	QRect rect = m_poly.boundingRect();

	if ((rect.width() == 0 || rect.height() == 0) ||
			(rect.right() == -1 || rect.bottom() == -1)) {
		return changed;
	}

	for(int img_y = rect.top(); img_y <= rect.bottom(); img_y++) {
		for(int img_x = rect.left(); img_x <= rect.right(); img_x++) {
			int u = img_x;
			int v = 255 - img_y;

            if (m_poly.containsPoint(QPoint(img_x, img_y), Qt::WindingFill)) {
				unsigned char oldValue = m_lookupTable[u*256 + v];
				unsigned char newValue = (m_add ? 255 : 0);
                if (oldValue != newValue) {
					changed = true;
					m_lookupTable[u*256 + v] = newValue;
					m_changes.push_back(new Change(u*256 + v, oldValue, newValue));
				}
			}
		}
	}

	return changed;
}

CommandImage2Lookuptable::CommandImage2Lookuptable(unsigned char * img, int width, int height, unsigned char * lookupTable, QPolygon poly, bool add, bool outer) {
	m_lookupTable = lookupTable;
	m_img = img;
    m_width = width;
	m_height = height;
	m_poly = poly;
	m_add = add;
    m_outer = outer;
}

bool CommandImage2Lookuptable::doIt() {
	bool changed = false;
	unsigned char * currImage = m_img;

    QRect rect = m_poly.boundingRect();
    int rectLeft = rect.left() * 2;
    if (m_outer) {
        std::list<int> containedValues;
        for (int img_y = rect.top(); img_y <= rect.bottom(); img_y++) {
            int u = -1, v = -1, y;
            for (int img_x = rectLeft; img_x <= rect.right() * 2; img_x += 2) {
                if ((img_x / 2) % 2 == 0)
                    u  = currImage[img_y * m_width * 2 + img_x];
                else
                    v  = currImage[img_y * m_width * 2 + img_x];
                y = currImage[img_y * m_width * 2 + img_x + 1];

                if (m_poly.containsPoint(QPoint(img_x / 2, img_y), Qt::WindingFill) &&
                        u != -1 && v != -1 &&
                        m_lookupTable[u*256 + v] > 0) {
                    containedValues.push_back(u*256 + v);
                }
            }
        }

        for (int img_y = 0; img_y < m_height; img_y++) {
            int u = -1, v = -1, y;
            for (int img_x = 0; img_x < m_width * 2; img_x += 2) {
                if ((img_x / 2) % 2 == 0)
                    u  = currImage[img_y * m_width * 2 + img_x];
                else
                    v  = currImage[img_y * m_width * 2 + img_x];
                y = currImage[img_y * m_width * 2 + img_x + 1];

                if (u != -1 && v != -1) {
                    if ((m_add && std::find(containedValues.begin(), containedValues.end(), u*256 + v) == containedValues.end()) ||
                            (!m_add && !m_poly.containsPoint(QPoint(img_x / 2, img_y), Qt::WindingFill))) {
                        unsigned char oldValue = m_lookupTable[u*256 + v];
                        unsigned char newValue = 0;
                        if (oldValue != newValue) {
                            changed = true;
                            m_lookupTable[u*256 + v] = newValue;
                            m_changes.push_back(new Change(u*256 + v, oldValue, newValue));
                        }
                    }
                }
            }
        }
    } else {
        for (int img_y = rect.top(); img_y <= rect.bottom(); img_y++) {
            int u = -1, v = -1, y;
            if (rect.width() == 1) {
                if (rectLeft > 0) {
                    int img_x = rectLeft - 2;
                    if ((img_x / 2) % 2 == 0)
                        u  = currImage[img_y * m_width * 2 + img_x];
                    else
                        v  = currImage[img_y * m_width * 2 + img_x];
                }
                if (rect.right() < m_width) {
                    rect.setRight(rect.right() + 1);
                }
            }
            for (int img_x = rectLeft; img_x <= rect.right() * 2; img_x += 2) {
                if ((img_x / 2) % 2 == 0)
                    u  = currImage[img_y * m_width * 2 + img_x];
                else
                    v  = currImage[img_y * m_width * 2 + img_x];
                y = currImage[img_y * m_width * 2 + img_x + 1];

                if (m_poly.containsPoint(QPoint(img_x / 2, img_y), Qt::WindingFill) &&
                        u != -1 && v != -1) {
                    unsigned char oldValue = m_lookupTable[u*256 + v];
                    unsigned char newValue = (m_add ? 255 : 0);
                    if (oldValue != newValue) {
                        changed = true;
                        m_lookupTable[u*256 + v] = newValue;
                        m_changes.push_back(new Change(u*256 + v, oldValue, newValue));
                    }
                }
            }
        }
    }

	return changed;
}

CommandInterpolateYUV2Lookuptable::CommandInterpolateYUV2Lookuptable(unsigned char * lookupTable, QPolygon poly, double threshold, unsigned int numberOfClusters) {
	m_lookupTable = lookupTable;
	m_poly = poly;
	m_threshold = threshold;
	m_numberOfClusters = numberOfClusters;
}

bool CommandInterpolateYUV2Lookuptable::doIt() {
	bool changed = false;

	QRect rect = m_poly.boundingRect();
	if ((rect.width() == 0 || rect.height() == 0) ||
			(rect.right() == -1 || rect.bottom() == -1)) {
		return changed;
	}

	EM *em = new EM(m_numberOfClusters, rect);

	for(int img_y=rect.top(); img_y <= rect.bottom(); img_y++) {
		for(int img_x=rect.left(); img_x <= rect.right(); img_x++) {
			int u = img_x;
			int v = 255 - img_y;

			if (m_poly.containsPoint(QPoint(img_x, img_y), Qt::WindingFill) &&
					m_lookupTable[u*256 + v] > 0) {
				dataType data;
				data(0) = img_x / 10.0;
				data(1) = img_y / 10.0;
				em->addData(data);
			}
		}
	}

	em->perform();

	double min_p = 1, max_p = 0;

	for(int img_y=rect.top(); img_y <= rect.bottom(); img_y++) {
		for(int img_x=rect.left(); img_x <= rect.right(); img_x++) {
			int u = img_x;
			int v = 255 - img_y;

			if (m_poly.containsPoint(QPoint(img_x, img_y), Qt::WindingFill) &&
					m_lookupTable[u*256 + v] == 0) {
				dataType data;
				data(0) = img_x / 10.0;
				data(1) = img_y / 10.0;

				for (list<Cluster*>::iterator it = em->m_cluster.begin(); it != em->m_cluster.end(); it++) {
					Cluster *cluster = *it;

					double p = em->pXC(data, cluster);
					if (p < min_p) {
						min_p = p;
					}
					if (p > max_p) {
						max_p = p;
					}
					if (p > m_threshold) {
						unsigned char oldValue = m_lookupTable[u*256 + v];
						unsigned char newValue = 255;
						changed = true;
						m_lookupTable[u*256 + v] = newValue;
						m_changes.push_back(new Change(u*256 + v, oldValue, newValue));
						break;
					}
				}
			}
		}
	}

	delete em;

	return changed;
}

CommandThresholds2Lookuptable::CommandThresholds2Lookuptable(unsigned char * lookupTable, unsigned char newLowerThreshold, unsigned char newUpperThreshold) {
	m_lookupTable = lookupTable;
	m_newLowerThreshold = newLowerThreshold;
	m_newUpperThreshold = newUpperThreshold;
}

bool CommandThresholds2Lookuptable::doIt() {
	bool changed = false;

	unsigned char oldValue;

	oldValue = m_lookupTable[256 * 256 + 0];
	if (oldValue != m_newLowerThreshold) {
		changed = true;
		m_lookupTable[256 * 256 + 0] = m_newLowerThreshold;
		m_changes.push_back(new Change(256 * 256 + 0, oldValue, m_newLowerThreshold));
	}

	oldValue = m_lookupTable[256 * 256 + 1];
	if (oldValue != m_newUpperThreshold) {
		changed = true;
		m_lookupTable[256 * 256 + 1] = m_newUpperThreshold;
		m_changes.push_back(new Change(256 * 256 + 1, oldValue, m_newUpperThreshold));
	}

	return changed;
}
