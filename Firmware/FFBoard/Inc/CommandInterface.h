/*
 * CommandInterface.h
 *
 *  Created on: 17.11.2021
 *      Author: Yannick
 */

#ifndef SRC_COMMANDINTERFACE_H_
#define SRC_COMMANDINTERFACE_H_

#include "CmdParser.h"
#include "FFBoardMainCommandThread.h"
#include "thread.hpp"
#include "CommandHandler.h"


class FFBoardMainCommandThread;

/**
 * Different command interface implementations
 */

class CommandInterface {
public:
	static std::vector<CommandInterface*> cmdInterfaces;

	CommandInterface();
	virtual ~CommandInterface();
	const virtual std::string getHelpstring(){return "";};
	virtual bool getNewCommands(std::vector<ParsedCommand>& commands) = 0;
	virtual bool hasNewCommands();
	static CommandResult makeCommandReply(const std::vector<CommandReply>& reply,CommandHandler* handler, uint32_t cmdId,CMDtype type = CMDtype::get,CommandInterface* originalInterface = nullptr);
	static void broadcastCommandReplyAsync(const std::vector<CommandReply>& reply,CommandHandler* handler, uint32_t cmdId,CMDtype type = CMDtype::get);
	virtual void sendReplies(const std::vector<CommandResult>& results,CommandInterface* originalInterface) = 0; // All commands from batch done
	virtual void sendReplyAsync(std::vector<CommandReply>& reply,CommandHandler* handler, uint32_t cmdId,CMDtype type);
	virtual bool readyToSend();
	virtual void batchDone();
protected:
	bool parserReady = false;
};

/**
 * Helper class for string based interfaces
 */
class StringCommandInterface : public CommandInterface{
public:
	StringCommandInterface(uint32_t bufferMaxCapacity = 512) : parser(CmdParser(bufferMaxCapacity)){}
	bool addBuf(char* Buf, uint32_t *Len);
	uint32_t bufferCapacity();
	bool getNewCommands(std::vector<ParsedCommand>& commands) override;
	static void formatReply(std::string& reply,const std::vector<CommandResult>& results,const bool formatWriteAsRead=false);
	static std::string formatOriginalCommandFromResult(const ParsedCommand& originalCommand,CommandHandler* commandHandler,const bool formatWriteAsRead=false);
	static void generateReplyValueString(std::string& replyPart,const CommandReply& reply);
	static void generateReplyFromCmd(std::string& replyPart,const ParsedCommand& originalCommand);
	const std::string getHelpstring(){return "Syntax:\nGet: cls.(instance.)cmd? or cls.(instance.)cmd?adr\nSet: cls.(instance.)cmd=val or cls.(instance.)cmd=val?adr";};


protected:
	CmdParser parser; // String parser
};


//receives bytes from mainclass. calls its own parser instance, calls global parser thread, passes replies  back to cdc port.
class CDC_CommandInterface : public StringCommandInterface,public cpp_freertos::Thread{
public:
	CDC_CommandInterface();
	virtual ~CDC_CommandInterface();

	void Run();

	void sendReplies(const std::vector<CommandResult>& results,CommandInterface* originalInterface) override;
	const std::string getHelpstring(){return "CDC interface. " + StringCommandInterface::getHelpstring();};
	bool readyToSend() override;
	void batchDone() override;

private:
	bool enableBroadcastFromOtherInterfaces = true;
	uint32_t lastSendTime = 0;
	const uint32_t parserTimeout = 500;
	//cpp_freertos::BinarySemaphore sendSem;
	std::vector<CommandResult> resultsBuffer;
	bool nextFormat = false;
	std::string sendBuffer;
	uint32_t bufferLength = 0;
	const uint32_t maxSendBuffer = 1024; // Max buffered command size before sending immediately
};

#endif /* SRC_COMMANDINTERFACE_H_ */
