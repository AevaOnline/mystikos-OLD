// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

#ifndef _MYST_OPTIONS_H
#define _MYST_OPTIONS_H

#include <myst/types.h>

typedef struct myst_options
{
    bool trace_syscalls;
    bool have_syscall_instruction;
    bool export_ramfs;
} myst_options_t;

extern myst_options_t __options;

#endif /* _MYST_OPTIONS_H */
