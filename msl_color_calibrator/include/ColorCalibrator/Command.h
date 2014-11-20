#ifndef COMMAND_H
#define COMMAND_H

#include <QObject>
#include <QRect>
#include <QPolygon>

#include <stack>
#include <list>

using namespace std;

class Change {
	public:
		Change(int nPosition, unsigned char nOldValue, unsigned char nNewValue);

		int position;
		unsigned char oldValue;
		unsigned char newValue;
};

class AbstractCommand {
	public:
		~AbstractCommand();

		virtual bool doIt() = 0;

		void undo();
		void redo();

	protected:
		unsigned char * m_lookupTable;
		list<Change*> m_changes;
};

class CommandList : public QObject {
		Q_OBJECT
	public:
		~CommandList();

		void addCommand(AbstractCommand* command);

		bool canUndo();
		bool canRedo();

		void undo();
		void redo();

		void clear();

	Q_SIGNALS:
		void listChanged();

	private:
		stack<AbstractCommand*> m_commands;
		stack<AbstractCommand*> m_redoMemory;
};

class CommandClearLookuptable : public AbstractCommand {
	public:
		CommandClearLookuptable(unsigned char * lookupTable, bool defaultValues);
		bool doIt();
	private:
		bool m_defaultValues;
};

class CommandYUV2Lookuptable : public AbstractCommand {
	public:
		CommandYUV2Lookuptable(unsigned char * lookupTable, QPolygon poly, bool add);
		bool doIt();
	private:
		QPolygon m_poly;
		bool m_add;
};

class CommandImage2Lookuptable : public AbstractCommand {
	public:
        CommandImage2Lookuptable(unsigned char * img, int width, int height, unsigned char * lookupTable, QPolygon poly, bool add, bool outer);
		bool doIt();
	private:
		unsigned char * m_img;
		int m_width;
		int m_height;
		QPolygon m_poly;
		bool m_add;
        bool m_outer;
};

class CommandInterpolateYUV2Lookuptable : public AbstractCommand {
	public:
		CommandInterpolateYUV2Lookuptable(unsigned char * lookupTable, QPolygon poly, double threshold, unsigned int numberOfClusters);
		bool doIt();
	private:
		QPolygon m_poly;
		double m_threshold;
		unsigned int m_numberOfClusters;
};

class CommandThresholds2Lookuptable : public AbstractCommand {
	public:
		CommandThresholds2Lookuptable(unsigned char * lookupTable, unsigned char newLowerThreshold, unsigned char newUpperThreshold);
		bool doIt();
	private:
		unsigned char m_newLowerThreshold;
		unsigned char m_newUpperThreshold;
};

#endif // COMMAND_H
