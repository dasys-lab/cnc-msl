/*
 * XMLProtocolParser.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Stephan Opfer
 */

#include "msl_refbox/XMLProtocolParser.h"
#include <iostream>

namespace msl_refbox
{

XMLProtocolParser::XMLProtocolParser(GameData *gameData)
{
    this->gameData = gameData;
    this->cyan = false;
    this->magenta = false;
}

XMLProtocolParser::~XMLProtocolParser()
{
    // TODO Auto-generated destructor stub
}

void XMLProtocolParser::handle(tinyxml2::XMLElement *element)
{
    processElement(element);
    tinyxml2::XMLElement *curChild = nullptr;
    tinyxml2::XMLElement *nextSibling = nullptr;

    curChild = element->FirstChildElement();
    nextSibling = element->NextSiblingElement();
    if (curChild != nullptr)
        handle(curChild);
    if (nextSibling != nullptr)
        handle(nextSibling);
}

void XMLProtocolParser::processElement(tinyxml2::XMLElement *curChild)
{
    const char *val = curChild->Value();
    std::string str(val);

    if (str.compare("Referee") == 0)
    {
        handleReferee(curChild);
    }
    else if (str.compare("StageChange") == 0)
    {
        handleStageChange(curChild);
    }
    else if (str.compare("GameStart") == 0)
    {
        handleGameStart(curChild);
    }
    else if (str.compare("GameStop") == 0)
    {
        handleGameStop(curChild);
    }
    else if (str.compare("KickOff") == 0)
    {
        handleKickOff(curChild);
    }
    else if (str.compare("DroppedBall") == 0)
    {
        handleDroppedBall(curChild);
    }
    else if (str.compare("FreeKick") == 0)
    {
        handleFreeKick(curChild);
    }
    else if (str.compare("GoalKick") == 0)
    {
        handleGoalKick(curChild);
    }
    else if (str.compare("ThrowIn") == 0)
    {
        handleThrowIn(curChild);
    }
    else if (str.compare("Corner") == 0)
    {
        handleCornerKick(curChild);
    }
    else if (str.compare("Penalty") == 0)
    {
        handlePenalty(curChild);
    }
    else if (str.compare("Parking") == 0)
    {
        handleParking(curChild);
    }
    else if (str.compare("RefboxEvent") == 0)
    {
        // DO NOTHING THIS CUZ OF --> <RefboxEvent xmlns="http://www.robocup.org/MSL/refbox/protocol2010">
    }
    else if (str.compare("GameInfo") == 0)
    {
        this->gameData->setGoals(GameData::Side::CYAN, 0);
        this->gameData->setGoals(GameData::Side::MAGENTA, 0);
        handleGameInfo(curChild);
    }
    else if (str.compare("TeamData") == 0)
    {
        handleTeamData(curChild);
    }
    else if (str.compare("Setup") == 0)
    {
        handleSetup(curChild);
    }
    else if (str.compare("Player") == 0)
    {
        handlePlayer(curChild);
    }
    else if (str.compare("PlayerOut") == 0)
    {
        handlePlayerOut(curChild);
    }
    else if (str.compare("PlayerIn") == 0)
    {
        handlePlayerIn(curChild);
    }
    else if (str.compare("GoalAwarded") == 0)
    {
        handleGoalAwarded(curChild);
    }
    else if (str.compare("CardAwarded") == 0)
    {
        handleCardAwarded(curChild);
    }
    else if (str.compare("Goals") == 0)
    {
        handleGoals(curChild);
    }
    else if (str.compare("Cards") == 0)
    {
        //                      Nothing to, comes with team data
    }
    else if (str.compare("PlayersOut") == 0)
    {
        //                      Currently ignored
    }
    else
    {
        std::cerr << "RQTREFBOX: CANNOT PARSE-> " << val << std::endl;
    }
}

/*##### Handle methods for all xml Elements #####*/

void XMLProtocolParser::handleReferee(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("time") == 0)
        {
            this->gameData->refBox->lbl_time->setText(QString(valAttr));
        }
        else if (strName.compare("stage") == 0)
        {
            this->gameData->refBox->lbl_stage->setText(QString(valAttr));
        }
        attr = attr->Next();
    }
}
void XMLProtocolParser::handleStageChange(tinyxml2::XMLElement *curChild)
{
    // TODO SUPER KACKE SOWAS
}
void XMLProtocolParser::handleGameStop(tinyxml2::XMLElement *curChild)
{
    this->gameData->sendStop(false);
}

void XMLProtocolParser::handleGameStart(tinyxml2::XMLElement *curChild)
{
    this->gameData->sendStart(false);
}

void XMLProtocolParser::handleKickOff(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    const char *val = attr->Value();
    std::string str(val);

    if (str.compare("Cyan") == 0)
    {
        this->gameData->sendCyanKickOff(false);
    }
    else if (str.compare("Magenta") == 0)
    {
        this->gameData->sendMagentaKickOff(false);
    }
}

void XMLProtocolParser::handleDroppedBall(tinyxml2::XMLElement *curChild)
{
    this->gameData->sendDroppedBall(false);
}

void XMLProtocolParser::handleFreeKick(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    const char *val = attr->Value();
    std::string str(val);

    if (str.compare("Cyan") == 0)
    {
        this->gameData->sendCyanFreeKick(false);
    }
    else if (str.compare("Magenta") == 0)
    {
        this->gameData->sendMagentaFreeKick(false);
    }
}

void XMLProtocolParser::handleGoalKick(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    const char *val = attr->Value();
    std::string str(val);

    if (str.compare("Cyan") == 0)
    {
        this->gameData->sendCyanGoalKick(false);
    }
    else if (str.compare("Magenta") == 0)
    {
        this->gameData->sendMagentaGoalKick(false);
    }
}

void XMLProtocolParser::handleThrowIn(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    const char *val = attr->Value();
    std::string str(val);

    if (str.compare("Cyan") == 0)
    {
        this->gameData->sendCyanThrowin(false);
    }
    else if (str.compare("Magenta") == 0)
    {
        this->gameData->sendMagentaThrownin(false);
    }
}

void XMLProtocolParser::handleCornerKick(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    const char *val = attr->Value();
    std::string str(val);

    if (str.compare("Cyan") == 0)
    {
        this->gameData->sendCyanCornerKick(false);
    }
    else if (str.compare("Magenta") == 0)
    {
        this->gameData->sendMagentaCornerKick(false);
    }
}

void XMLProtocolParser::handlePenalty(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    const char *val = attr->Value();
    std::string str(val);

    if (str.compare("Cyan") == 0)
    {
        this->gameData->sendCyanPenalty(false);
    }
    else if (str.compare("Magenta") == 0)
    {
        this->gameData->sendMagentaPenalty(false);
    }
}

void XMLProtocolParser::handleParking(tinyxml2::XMLElement *curChild)
{
    this->gameData->sendParking(false);
}

void XMLProtocolParser::handleGameInfo(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("time") == 0)
        {
            this->gameData->refBox->lbl_time->setText(QString(valAttr));
        }
        else if (strName.compare("stage") == 0)
        {
            this->gameData->refBox->lbl_stage->setText(QString(valAttr));
        }
        attr = attr->Next();
    }
}

void XMLProtocolParser::handleTeamData(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("team") == 0)
        {
            if (strAttr.compare("Cyan") == 0)
            {
                this->cyan = true;
                this->magenta = false;
            }
            else if (strAttr.compare("Magenta") == 0)
            {
                this->cyan = false;
                this->magenta = true;
            }
        }
        attr = attr->Next();
    }
}

void XMLProtocolParser::handleGoals(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    GameData::Side side = GameData::Side::ALL;

    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("team") == 0)
        {
            if (strAttr.compare("Cyan") == 0)
            {
                side = GameData::Side::CYAN;
            }
            else if (strAttr.compare("Magenta") == 0)
            {
                side = GameData::Side::MAGENTA;
            }
        }
        attr = attr->Next();
    }

    if (side != GameData::Side::ALL)
    {
        this->gameData->setGoals(side, this->gameData->getGoals(side) + 1);
    }
}

void XMLProtocolParser::handleSetup(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("name") == 0)
        {
            fillSetup(valAttr);
        }
        else if (strName.compare("leader") == 0)
        {
            fillSetup(valAttr);
        }

        attr = attr->Next();
    }
}

void XMLProtocolParser::handlePlayer(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    std::string name = "";
    std::string inField = "";
    bool found = false;
    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("name") == 0)
        {
            found = fillSetup(valAttr);
            name = valAttr;
        }
        else if (strName.compare("inField") == 0)
        {
            inField = valAttr;
        }

        attr = attr->Next();
    }
    if (!found)
        fillTable(name, inField);
}

void XMLProtocolParser::handlePlayerOut(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    GameData::Side side;
    std::string reason;

    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("team") == 0)
        {
            if (strAttr.compare("Cyan") == 0)
            {
                side = GameData::Side::CYAN;
            }
            else if (strAttr.compare("Magenta") == 0)
            {
                side = GameData::Side::MAGENTA;
            }
        }
        else if (strName.compare("reason") == 0)
        {
            reason = strAttr;
        }

        attr = attr->Next();
    }

    this->gameData->log("Player out " + reason, false, side);
}

void XMLProtocolParser::handlePlayerIn(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    GameData::Side side;
    std::string reason;

    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("team") == 0)
        {
            if (strAttr.compare("Cyan") == 0)
            {
                side = GameData::Side::CYAN;
            }
            else if (strAttr.compare("Magenta") == 0)
            {
                side = GameData::Side::MAGENTA;
            }
        }
        else if (strName.compare("reason") == 0)
        {
            reason = strAttr;
        }

        attr = attr->Next();
    }

    this->gameData->log("Player in " + reason, false, side);
}

void XMLProtocolParser::handleGoalAwarded(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("team") == 0)
        {
            if (strAttr.compare("Cyan") == 0)
            {
                this->gameData->setGoals(GameData::Side::CYAN, this->gameData->getGoals(GameData::Side::CYAN) + 1);
                this->gameData->log("Goal", false, GameData::Side::CYAN);
            }
            else if (strAttr.compare("Magenta") == 0)
            {
                this->gameData->setGoals(GameData::Side::MAGENTA, this->gameData->getGoals(GameData::Side::MAGENTA) + 1);
                this->gameData->log("Goal", false, GameData::Side::MAGENTA);
            }
        }
        else if (strName.compare("player") == 0)
        {
            // TODO NEUER TAB
        }
        else if (strName.compare("own") == 0)
        {
            // TODO NEUER TAB
        }
        else if (strName.compare("stage") == 0)
        {
            this->gameData->refBox->lbl_stage->setText(QString(valAttr));
        }
        else if (strName.compare("time") == 0)
        {
            this->gameData->refBox->lbl_time->setText(QString(valAttr));
        }
        attr = attr->Next();
    }
}

void XMLProtocolParser::handleCardAwarded(tinyxml2::XMLElement *curChild)
{
    const tinyxml2::XMLAttribute *attr = curChild->FirstAttribute();
    std::string team = "";
    std::string player = "";
    std::string color = "";
    std::string number = "";

    while (attr != nullptr)
    {
        const char *valAttr = attr->Value();
        const char *valName = attr->Name();
        std::string strAttr(valAttr);
        std::string strName(valName);

        if (strName.compare("team") == 0)
        {
            team = strAttr;
        }
        else if (strName.compare("player") == 0)
        {
            player = strAttr;
        }
        else if (strName.compare("color") == 0)
        {
            color = strAttr;
        }
        else if (strName.compare("number") == 0)
        {
            number = strAttr;
        }
        else if (strName.compare("time") == 0)
        {
            this->gameData->refBox->lbl_time->setText(QString(valAttr));
        }
        else if (strName.compare("stage") == 0)
        {
            this->gameData->refBox->lbl_stage->setText(QString(valAttr));
        }
        attr = attr->Next();
    }
    fillYellow(team, player, color, number);
}
bool XMLProtocolParser::fillSetup(const char *valAttr)
{
    if (cyan && std::find(cyanSetup.begin(), cyanSetup.end(), valAttr) == cyanSetup.end())
    {
        cyanSetup.push_back(valAttr);
        return false;
    }
    else if (magenta && std::find(magentaSetup.begin(), magentaSetup.end(), valAttr) == magentaSetup.end())
    {
        magentaSetup.push_back(valAttr);
        return false;
    }
    return true;
}
void XMLProtocolParser::fillTable(std::string name, std::string inField)
{
    int row = this->gameData->refBox->tbl_info->rowCount();
    std::string team = "";
    if (cyan)
        team = cyanSetup[0];
    else if (magenta)
        team = magentaSetup[0];

    for (int i = 0; i < row; i++)
    {
        if (this->gameData->refBox->tbl_info->item(i, 0) == nullptr)
        {
            QTableWidgetItem *item = new QTableWidgetItem();
            item->setText(QString::fromStdString(team));
            this->gameData->refBox->tbl_info->setItem(i, 0, item);

            QTableWidgetItem *item1 = new QTableWidgetItem();
            item1->setText(QString::fromStdString(name));
            this->gameData->refBox->tbl_info->setItem(i, 1, item1);

            QTableWidgetItem *item2 = new QTableWidgetItem();
            item2->setText(QString::fromStdString(inField));
            this->gameData->refBox->tbl_info->setItem(i, 2, item2);

            QTableWidgetItem *item3 = new QTableWidgetItem();
            this->gameData->refBox->tbl_info->setItem(i, 3, item3);

            if (cyan)
            {
                item->setBackground(Qt::cyan);
                item1->setBackground(Qt::cyan);
                item2->setBackground(Qt::cyan);
                item3->setBackground(Qt::cyan);
            }
            else
            {
                item->setBackground(Qt::magenta);
                item1->setBackground(Qt::magenta);
                item2->setBackground(Qt::magenta);
                item3->setBackground(Qt::magenta);
            }

            break;
        }
    }
}
void XMLProtocolParser::fillYellow(std::string team, std::string player, std::string color, std::string number)
{
    int row = this->gameData->refBox->tbl_info->rowCount();
    std::string teamName = "";
    GameData::Side side;
    if (team.compare("Cyan") == 0)
    {
        teamName = cyanSetup[0];
        side = GameData::Side::CYAN;
    }
    else
    {
        teamName = magentaSetup[0];
        side = GameData::Side::MAGENTA;
    }

    if (color == "yellow")
    {
        this->gameData->log("Yellow Card (" + number + ") for " + player, false, side);
    }
    else
    {
        this->gameData->log("Red Card for " + player, false, side);
    }

    for (int i = 0; i < row; i++)
    {
        if (this->gameData->refBox->tbl_info->item(i, 1) != nullptr)
        {
            if (this->gameData->refBox->tbl_info->item(i, 1)->text().compare(QString::fromStdString(player)) == 0 &&
                this->gameData->refBox->tbl_info->item(i, 0)->text().compare(QString::fromStdString(teamName)) == 0)
            {
                if (number.compare("1") == 0 && color.compare("yellow") == 0)
                {
                    this->gameData->refBox->tbl_info->item(i, 3)->setBackground(Qt::yellow);
                    this->gameData->refBox->tbl_info->item(i, 3)->setText("first yellow");
                }
                else if (number.compare("2") == 0 && color.compare("yellow") == 0)
                {
                    this->gameData->refBox->tbl_info->item(i, 3)->setBackground(Qt::red);
                    this->gameData->refBox->tbl_info->item(i, 3)->setText("2 times yellow");
                }
                else
                {
                    this->gameData->refBox->tbl_info->item(i, 3)->setBackground(Qt::red);
                }
                break;
            }
        }
    }
}
/* namespace rqt_pm_control */
}
