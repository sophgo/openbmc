/*
 * Copyright (c) 2022 ASPEED Technology Inc.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <openssl/crypto.h>
#include <openssl/ec.h>
#include <openssl/pem.h>
#include <unistd.h>
#include "i2c_utils.h"
#include "mailbox_enums.h"
#include "arguments.h"

void checkpointStart(ARGUMENTS args)
{
	i2cWriteByteData(args, MB_BMC_CHECKPOINT, MB_CHKPT_START);
}

void checkpointPause(ARGUMENTS args)
{
	i2cWriteByteData(args, MB_BMC_CHECKPOINT, MB_CHKPT_PAUSE);
}

void checkpointResume(ARGUMENTS args)
{
	i2cWriteByteData(args, MB_BMC_CHECKPOINT, MB_CHKPT_RESUME);
}

void checkpointComplete(ARGUMENTS args)
{
	i2cWriteByteData(args, MB_BMC_CHECKPOINT, MB_CHKPT_COMPLETE);
}

void checkpoint(ARGUMENTS args)
{
	if (strncmp(args.checkpoint_cmd, "start", strlen("start")) == 0)
		checkpointStart(args);
	else if (strncmp(args.checkpoint_cmd, "pause", strlen("pause")) == 0)
		checkpointPause(args);
	else if (strncmp(args.checkpoint_cmd, "resume", strlen("resume")) == 0)
		checkpointResume(args);
	else if (strncmp(args.checkpoint_cmd, "complete", strlen("complete")) == 0)
		checkpointComplete(args);
	else
		printf("unsupported command, %s\n", args.checkpoint_cmd);
}

