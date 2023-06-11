#ifndef DATACOLLECTAGENT_H
#define DATACOLLECTAGENT_H

#include "DataCollectAgentAPI.h"


class DataCollectAgent : public I_DataCollectAgent
{
public:
    DataCollectAgent( void );
    ~DataCollectAgent( void );

    virtual void ProcessUserKey( UserKeyIDs UserKey );
    bool Process( void* pPluginImpl );
    void CreateDisplayPanels();
    void CreateDisplayViews( bool );
    void CreatePluginOwnedViews()
    {
        return;
    }
    char* GetActiveDisplayView( int )
    {
        return NULL;
    }
    void RegisterOCImpl( class COnlineCalibrationImpl* pOCImpl );


    class CAppCtrlInfo* m_pAppCtrl;

private:
#define MAX_NUM_OF_GRAPHS       150
    GraphStruct m_GraphPool[MAX_NUM_OF_GRAPHS];

};

#endif // !DATACOLLECTAGENT_H

