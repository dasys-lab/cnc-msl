/*
 * CNMCPacketRequest.cpp
 *
 *  Created on: Sep 30, 2015
 *      Author: cnpaul
 */

#include <CNMCPacket.h>

namespace msl_driver
{

	const int CNMCPacket::CMD_GROUP_POS = 1;
	const int CNMCPacket::CMD_POS = 2;
	const int CNMCPacket::DATA_POS = 3;
	const uint8_t CNMCPacket::QUOTE = 0x84;
	const uint8_t CNMCPacket::START_HEADER = 0x81;
	const uint8_t CNMCPacket::END_HEADER = 0x82;

	CNMCPacket::CNMCPacket()
	{
		this->data = std::make_shared<std::vector<uint8_t> >();
	}

	CNMCPacket::~CNMCPacket()
	{
	}
	std::shared_ptr<std::vector<uint8_t>> CNMCPacket::getBytes()
	{
		std::shared_ptr<std::vector<uint8_t>> dataList = std::make_shared<std::vector<uint8_t>>();

		dataList->push_back(START_HEADER);
		needQuotes(this->cmdgrp, dataList);
		needQuotes(this->cmd, dataList);

		for (int i = 0; i < this->data->size(); i++)
		{
			uint8_t b = this->data->at(i);
			dataList->push_back(b);
		}
		dataList->push_back(END_HEADER);
		return dataList;

	}
	void CNMCPacket::needQuotes(uint8_t b, std::shared_ptr<std::vector<uint8_t>> list)
	{

		if (b == START_HEADER || b == END_HEADER || b == QUOTE)
		{
			list->push_back(QUOTE);
			list->push_back(b);
		}
		else
		{
			list->push_back(b);
		}
	}

	bool CNMCPacket::isExpectedResponse(std::unique_ptr<CNMCPacket> response)
	{
		bool retVal = (this->cmd == response->cmd && this->cmdgrp + 1 == response->cmdgrp
				&& this->data->size() == response->data->size());
		for (int i = 0; i < this->data->size() && retVal; i++)
		{
			retVal &= this->data->at(i) == response->data->at(i);
		}
		return retVal;
	}

	std::unique_ptr<CNMCPacket> CNMCPacket::getInstance(uint8_t raw[], int size)
	{
		if (size < 3)
		{
			std::cerr << "Invalid CNMC packet" << std::endl;
		}
		switch ((CNMCPacket::CommandGroup)raw[CNMCPacket::CMD_GROUP_POS])
		{
			case CNMCPacket::CommandGroup::Configure:
				return std::move(std::unique_ptr<CNMCPacket>(new CNMCPacketConfigure(raw)));

			case CNMCPacket::CommandGroup::Control:
				return std::move(std::unique_ptr<CNMCPacket>(new CNMCPacketControl(raw)));

			case CNMCPacket::CommandGroup::Request:
				return std::move(std::unique_ptr<CNMCPacket>(new CNMCPacketRequest(raw)));

			case CNMCPacket::CommandGroup::CtrlConfigure:
				return std::move(std::unique_ptr<CNMCPacket>(new CNMCPacketCtrlConfigure(raw)));

			case CNMCPacket::CommandGroup::CtrlConfigureResponse:
				return std::move(std::unique_ptr<CNMCPacket>(new CNMCPacketCtrlConfigureResponse(raw)));

			case CNMCPacket::CommandGroup::ErrorResponse:
				return std::move(std::unique_ptr<CNMCPacket>(new CNMCPacketError(raw)));

			case CNMCPacket::CommandGroup::RequestResponse:
				return std::move(std::unique_ptr<CNMCPacket>(new CNMCPacketRequestResponse(raw, size)));

			case CNMCPacket::CommandGroup::ConfigureResponse:
				return std::move(std::unique_ptr<CNMCPacket>(new CNMCPacketConfigureResponse(raw, size)));

			default:
				std::cerr << "UNKNOWN CNMC packet" << std::endl;
				return std::move(std::unique_ptr<CNMCPacket>());
		}
	}

	void CNMCPacket::add(uint8_t b)
	{
		if ((b == QUOTE) || (b == START_HEADER) || (b == END_HEADER))
		{
			this->data->push_back(QUOTE);
		}
		this->data->push_back(b);
	}

	void CNMCPacket::add(std::vector<uint8_t> bytes)
	{
		for (uint8_t b : bytes)
		{
			if ((b == QUOTE) || (b == START_HEADER) || (b == END_HEADER))
			{
				this->data->push_back(QUOTE);
			}
			this->data->push_back(b);
		}
	}

	//**** CONVERT METHODS ****//
//	CNMCPacket::VAROBJECT CNMCPacket::convertByteToShort(uint8_t data[], int start)
//	{
//		short sh = (short)(((unsigned short)(data[start]) << 8 ) + (unsigned char)data[start+1]);
//
//		VAROBJECT o = VAROBJECT();
//		o.value.shValue = sh;
//		o.objectType = VAROBJECT::SHORT;
//
//		return o;
//
//	}
	std::vector<uint8_t> CNMCPacket::convertShortToByte(short data)
	{
		std::vector<uint8_t> byteVector;
		byteVector.push_back(data & 0xff);
		byteVector.push_back((data >> 8) & 0xff);
		return byteVector;
	}
//	CNMCPacket::VAROBJECT CNMCPacket::convertByteToInt(uint8_t data[], int start)
//	{
//		int i = 0;
//		i = (i << 8) + data[start+3];
//		i = (i << 8) + data[start+2];
//		i = (i << 8) + data[start+1];
//		i = (i << 8) + data[start];
//
//
//		VAROBJECT o = VAROBJECT();
//		o.value.dblValue = i;
//		o.objectType = VAROBJECT::INT;
//		return o;
//	}
//
	std::vector<uint8_t> CNMCPacket::convertIntToByte(int data)
	{
		std::vector<uint8_t> byteVector;
		byteVector.push_back((data >> (3 * 8)) & 0xff);
		byteVector.push_back((data >> (2 * 8)) & 0xff);
		byteVector.push_back((data >> (1 * 8)) & 0xff);
		byteVector.push_back(data & 0xff);
		return byteVector;
	}

	//**** CONVERT METHODS END ****//

	///############## CNMCPacketConfigure ##############///
	CNMCPacketConfigure::CNMCPacketConfigure()
	{
		this->cmdgrp = (uint8_t)CommandGroup::Configure;
	}
	CNMCPacketConfigure::~CNMCPacketConfigure()
	{
	}
	CNMCPacketConfigure::CNMCPacketConfigure(uint8_t raw[])
	{
		if ((CNMCPacket::CommandGroup)raw[CNMCPacket::CMD_GROUP_POS] != CommandGroup::Configure)
		{
			std::cerr << "No configuration packet!" << std::endl;
		}
		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];
	}
	void CNMCPacketConfigure::setData(ConfigureCmd cmd, uint8_t val)
	{
		this->cmd = (uint8_t)cmd;
		this->data->clear();
		add(val);
	}
	void CNMCPacketConfigure::setData(ConfigureCmd cmd, short val)
	{
		this->cmd = (uint8_t)cmd;
		this->data->clear();
		add(convertShortToByte(val));
	}
	void CNMCPacketConfigure::setData(ConfigureCmd cmd, std::shared_ptr<std::vector<uint8_t>> vals)
	{
		this->cmd = (uint8_t) cmd;
		this->data->clear();

		for(int i = 0; i < vals->size(); i++)
		{
			uint8_t b = vals->at(i);
			add(b);
		}
	}
	void CNMCPacketConfigure::setData(ConfigureCmd cmd, std::shared_ptr<std::vector<int8_t>> vals)
	{
		this->cmd = (uint8_t) cmd;
		this->data->clear();
		for(int i = 0; i < vals->size(); i++)
		{
			int8_t b = vals->at(i);
			add(b);
		}
	}
	///############## CNMCPacketControl ##############///
	CNMCPacketControl::CNMCPacketControl()
	{
		this->cmdgrp = (uint8_t)CommandGroup::Control;
	}
	CNMCPacketControl::~CNMCPacketControl()
	{
	}
	CNMCPacketControl::CNMCPacketControl(uint8_t raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::Control)
		{
			std::cerr << "No control packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		if (this->cmd != (uint8_t)ControlCmd::SetAllRPM)
		{
			std::cerr << "Unsuppported reply (Control): " << std::endl;
		}

//		this->values->clear();
//
//		VAROBJECT o1 = convertByteToShort(raw, DATA_POS);
//		this->values->push_back(o1);
//
//		VAROBJECT o2 = convertByteToShort(raw, DATA_POS+2);
//		this->values->push_back(o2);
//
//		VAROBJECT o3 = convertByteToShort(raw, DATA_POS+4);
//		this->values->push_back(o3);
//
//		VAROBJECT o4 = VAROBJECT();
//		o4.value.ucValue[0] = raw[DATA_POS+6];
//		o4.objectType = VAROBJECT::UNSIGNEDCHAR;
//		this->values->push_back(o4);
	}
	void CNMCPacketControl::setData(ControlCmd cmd, short x, short y, short rot)
	{
		this->cmd = (uint8_t)cmd;

		this->data->clear();

		add(convertShortToByte(x));
		add(convertShortToByte(y));
		add(convertShortToByte(rot));
	}

	///############## CNMCPacketRequest ##############///
	CNMCPacketRequest::CNMCPacketRequest()
	{
		this->cmdgrp = (uint8_t)CommandGroup::Request;
	}
	CNMCPacketRequest::~CNMCPacketRequest()
	{
	}
	CNMCPacketRequest::CNMCPacketRequest(uint8_t raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::Request)
		{
			std::cerr << "No request packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

//		this->values->clear();
//
//		if (this->cmd < 0x40)
//		{
//			VAROBJECT o4 = VAROBJECT();
//			o4.value.ucValue[0] = raw[DATA_POS];
//			o4.objectType = VAROBJECT::UNSIGNEDCHAR;
//			this->values->push_back(o4);
//
//			VAROBJECT o2 = convertByteToShort(raw, DATA_POS+1);;
//			this->values->push_back(o2);
//
//			if (raw[DATA_POS] == 0)
//			{
//				VAROBJECT o3 = convertByteToShort(raw, DATA_POS+3);
//				this->values->push_back(o3);
//
//				VAROBJECT o5 = convertByteToShort(raw, DATA_POS+5);
//				this->values->push_back(o5);
//			}
//
//		}
//		else if ((this->cmd & 0xfe) == 0x40)
//		{
//			VAROBJECT o4 = VAROBJECT();
//			o4.value.ucValue[0] = raw[DATA_POS];
//			o4.objectType = VAROBJECT::UNSIGNEDCHAR;
//			this->values->push_back(o4);
//
//			VAROBJECT o = VAROBJECT();
//			o.value.ucValue[0]  = raw[DATA_POS+1];
//			o.objectType = VAROBJECT::UNSIGNEDCHAR;
//			this->values->push_back(o);
//
//		}
//		else if (this->cmd == 0x42)
//		{
//			VAROBJECT o4 = VAROBJECT();
//			o4.value.ucValue[0]  = raw[DATA_POS];
//			o4.objectType = VAROBJECT::UNSIGNEDCHAR;
//			this->values->push_back(o4);
//
//			VAROBJECT o3 = convertByteToShort(raw, DATA_POS+1);
//			this->values->push_back(o3);
//
//			VAROBJECT o5 = convertByteToShort(raw, DATA_POS+3);
//			this->values->push_back(o5);
//
//
//		}
//		else
//		{
//			std::cerr << "Unsuppported reply (Request): " << std::endl;
//		}
	}
	void CNMCPacketRequest::setData(RequestCmd cmd)
	{
		this->cmd = (uint8_t)cmd;
		this->data->clear();
	}
	void CNMCPacketRequest::setData(RequestCmd cmd, short val)
	{
		this->cmd = (uint8_t)cmd;
		this->data->clear();
		add(convertShortToByte(val));
	}

	///############## CNMCPacketCtrlConfigureResponse ##############///
	CNMCPacketCtrlConfigureResponse::CNMCPacketCtrlConfigureResponse()
	{
		this->cmdgrp = (uint8_t)CommandGroup::CtrlConfigureResponse;
	}
	CNMCPacketCtrlConfigureResponse::~CNMCPacketCtrlConfigureResponse()
	{
	}
	CNMCPacketCtrlConfigureResponse::CNMCPacketCtrlConfigureResponse(uint8_t raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::CtrlConfigureResponse)
		{
			std::cerr << "No control configure response packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		if ((CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::Commit)
			return;
		if ((CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::MaxAcceleration
				|| (CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::MaxDecceleration
				|| (CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::MaxRotForce)
		{
			for (int i = 0; i < 4; i++)
			{
				add(raw[DATA_POS + i]);
			}
			return;
		}
		add(raw[DATA_POS]);
		add(raw[DATA_POS + 1]);

		if ((CtrlConfigureCmd)raw[CMD_POS] == CtrlConfigureCmd::FailSafeValues)
		{
			add(raw[DATA_POS + 2]);
			add(raw[DATA_POS + 3]);

			add(raw[DATA_POS + 4]);
			add(raw[DATA_POS + 5]);
		}

	}

	///############## CNMCPacketCtrlConfigure ##############///
	CNMCPacketCtrlConfigure::CNMCPacketCtrlConfigure()
	{
		this->cmdgrp = (uint8_t)CommandGroup::CtrlConfigure;
	}
	CNMCPacketCtrlConfigure::~CNMCPacketCtrlConfigure()
	{

	}
	CNMCPacketCtrlConfigure::CNMCPacketCtrlConfigure(uint8_t raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::CtrlConfigure)
		{
			std::cerr << "No control configure packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];
//
//		this->values->clear();
//
//		VAROBJECT o3 = giveVarobject(raw[DATA_POS], VAROBJECT::UNSIGNEDCHAR);
//		this->values->push_back(o3);
//
//		VAROBJECT o = convertByteToShort(raw,DATA_POS);
//		this->values->push_back(o);

	}
	void CNMCPacketCtrlConfigure::setData(CtrlConfigureCmd cmd)
	{
		this->cmd = (uint8_t)cmd;
		this->data->clear();
	}
	void CNMCPacketCtrlConfigure::setData(CtrlConfigureCmd cmd, short val)
	{
		this->cmd = (uint8_t)cmd;
		this->data->clear();
		add(convertShortToByte(val));

//		VAROBJECT o = VAROBJECT();
//		o.value.shValue = val;
//		o.objectType = VAROBJECT::SHORT;
//		this->values->push_back(o);

	}
	void CNMCPacketCtrlConfigure::setData(CtrlConfigureCmd cmdl, int val)
	{
		this->cmd = (uint8_t)cmd;
		this->data->clear();
		add(convertIntToByte(val));

//		VAROBJECT o = VAROBJECT();
//		o.value.dblValue = val;
//		o.objectType = VAROBJECT::INT;
//		this->values->push_back(o);

	}
	void CNMCPacketCtrlConfigure::setData(CtrlConfigureCmd cmd, std::shared_ptr<std::vector<short>> vals)
	{
		this->cmd = (uint8_t)cmd;
		this->data->clear();
		for (int i = 0; i < vals->size(); i++)
		{
			add(convertShortToByte(vals->at(i)));
//			VAROBJECT o = giveVarobject(vals->at(i), VAROBJECT::SHORT);
//			this->values->push_back(o);
		}

	}

	///############## CNMCPacketError ##############///

	CNMCPacketError::CNMCPacketError()
	{
		this->cmdgrp = (uint8_t)CommandGroup::ErrorResponse;
	}
	CNMCPacketError::~CNMCPacketError()
	{
	}
	CNMCPacketError::CNMCPacketError(uint8_t raw[])
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::ErrorResponse)
		{
			std::cerr << "No error packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		std::cerr << "No error packet!" << this->toString() << std::endl;
	}
	std::string CNMCPacketError::toString()
	{
		std::string reason = "";
		switch ((ErrorCmd)this->cmd)
		{
			case ErrorCmd::CycleOverTime:
				reason = "cycle overtime";
				break;
			case ErrorCmd::UnknownError:
				reason = "unknown";
				break;
			case ErrorCmd::UnknownCmd:
				reason = "unknown Cmd";
				break;
			case ErrorCmd::ParamError:
				reason = "Parameter Error";
				break;
			case ErrorCmd::OutOfRange:
				reason = "parameter out of range";
				break;
			default:
				break;
		}
		return std::string("ErrorReponse: ") + std::to_string(static_cast<unsigned>(this->cmd)) + std::string("(")
				+ reason + std::string(")");
	}

	///############## CNMCPacketRequestResponse ##############///
	CNMCPacketRequestResponse::CNMCPacketRequestResponse()
	{
		this->cmdgrp = (uint8_t)CommandGroup::RequestResponse;
	}

	CNMCPacketRequestResponse::~CNMCPacketRequestResponse()
	{
	}

	CNMCPacketRequestResponse::CNMCPacketRequestResponse(uint8_t raw[], int size)
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::RequestResponse)
		{
			std::cerr << "No RequestResponse packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		switch (this->cmd)
		{
			case RequestCmd::PathVector:
				add(raw[DATA_POS]);
				add(raw[DATA_POS + 1]);
				add(raw[DATA_POS + 2]);
				add(raw[DATA_POS + 3]);
				add(raw[DATA_POS + 4]);
				add(raw[DATA_POS + 5]);
				break;

			case RequestCmd::PathAndPWM:
				add(raw[DATA_POS]);
				add(raw[DATA_POS + 1]);
				add(raw[DATA_POS + 2]);
				add(raw[DATA_POS + 3]);
				add(raw[DATA_POS + 4]);
				add(raw[DATA_POS + 5]);
				add(raw[DATA_POS + 6]);
				add(raw[DATA_POS + 7]);
				add(raw[DATA_POS + 8]);
				add(raw[DATA_POS + 9]);
				add(raw[DATA_POS + 10]);
				add(raw[DATA_POS + 11]);
				break;

			case RequestCmd::EncoderTicksRel:
				add(raw[DATA_POS]);
				add(raw[DATA_POS + 1]);
				add(raw[DATA_POS + 2]);
				add(raw[DATA_POS + 3]);
				add(raw[DATA_POS + 4]);
				add(raw[DATA_POS + 5]);
				add(raw[DATA_POS + 6]);
				add(raw[DATA_POS + 7]);
				add(raw[DATA_POS + 8]);
				add(raw[DATA_POS + 9]);
				add(raw[DATA_POS + 10]);
				add(raw[DATA_POS + 11]);
				break;

			case RequestCmd::MotorCurrent:
				add(raw[DATA_POS]);
				add(raw[DATA_POS + 1]);
				add(raw[DATA_POS + 2]);
				add(raw[DATA_POS + 3]);
				add(raw[DATA_POS + 4]);
				add(raw[DATA_POS + 5]);
				break;

			case RequestCmd::BatteryVoltage:
				add(raw[DATA_POS]);
				add(raw[DATA_POS + 1]);
				break;

			case RequestCmd::ReadOdoLog:
				for (int i = DATA_POS; i < size; i++)
				{
					add(raw[i]);
				}
				break;

			default:
				std::cerr << "Never saw this response package" << std::endl;
		}
	}

	///############## CNMCPacketConfigureResponse ##############///
	CNMCPacketConfigureResponse::CNMCPacketConfigureResponse()
	{
		this->cmdgrp = (uint8_t)CommandGroup::ConfigureResponse;
	}

	CNMCPacketConfigureResponse::~CNMCPacketConfigureResponse()
	{
	}

	CNMCPacketConfigureResponse::CNMCPacketConfigureResponse(uint8_t raw[], int size)
	{
		if ((CommandGroup)raw[CMD_GROUP_POS] != CommandGroup::ConfigureResponse)
		{
			std::cerr << "No ConfigureResponse packet!" << std::endl;
		}

		this->cmdgrp = raw[CMD_GROUP_POS];
		this->cmd = raw[CMD_POS];

		for (int i = DATA_POS; i < size; i++)
		{
			add(raw[i]);
		}
	}
} /* namespace msl_driver */
