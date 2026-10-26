# Product Brief for the KMR Robotics Architecture (v3)

## Overview

### One-liner value proposition:

The most convinent robotics architecture focused on easy yet powerful usage
(API with excellent DX) that allows you to focus on your work and not on
time-consuming hardware implementation.

### What is it ?

The KMR architecture is designed from the ground-up with quality-of-life in mind.
Most software in robotics right now are a set of distinct librairies that
force you to redo every basic step (such as defining the control loop, or the
hardware implementation) over and over again. At best, you copy-paste and at
worst, you redo the whole code because you hit an implemantation incompatibilty.

What if you were able to use the same architecture, professionnaly designed by
experts, while still being able to do everything you ever wanted with no
restrictions ?

The architecture handles complex stuff for you such as hardware integration
and communication while still allowing you to create your own if you wish so.

The complex, but general concepts like the control loop are handled internally.
You don't write what you already wrote too many times, you configure it.

## Problem

The robotics industry gave up on improving its software stack.

- Huge set of librairies scattered around yet all codes are different.
- Scattered tools, mostly incompatible from one another
- Heavy redesign/refactor requirement at the slightest architectural change
- Tones of papers arguing about how the software and control should look like.

## Solution

We provide you with an extensible architecture, backed by our experience, built
from scratch with a safe modern language with fresh redesigned utilities
designed with robotics in mind from the start.

No longer do we need to rely on 30-year-old technology.

- We provide you with a unique interface that always looks the same, yet can be
customized with little effort.

- Tools are not scattered, they are centralized and compatible with any of yours
with minimal effort.

- A breaking bugs aren't silent. They're clearly defined with human-readable text,
not walls of alien text only machines or AI understand.

- You find a breaking bug ? It's fixed in a week, not in month. We never assumed
a perfect architecture from the start, we designed it to be easy to fix from our
side, so that you don't have to worry about the complexity you shouldn't care
about. Good architectural design is to plan ahead on how to fix problems, not
work around them.

- Don't waste time arguing about software design. Focus on what matters to you,
and worry about design once it for all, or for each project.
We don't restrain software design from your side. We just decided the minimal
amount, so you worry as little as possible about it. Furthermore, we guarantee enough
performance so that mild errors on your part doesn't need an optimization or
a paper. It's compensated, and you get for free advices from our documentation
about good practicies.

## Key Features

### Convinient API

API is the way you use our architecture.

From the ground-up, we designed the most convinient API imaginable that allows
maximal freedom and comfort.

Our API read like english, but respects high standards while using a very
powerful but complex language. But don't worry about complexity, you won't see
any of it.

You don't like writing in the language we choose ?
Well, we let you decide any language you want though our API. Write the minimum
to use our product, and write all the stay in plain python if you wish so.

### Memory safety

Most robotics controllers, the software the controlls the motion, are written
in an aging language. It is so inconvinient to use that the NASA, the US Air
Force and other powerful entities all have extremely tight documentation to
disallow features and explain to new developer what to never do within their
walls.

Instead of handing you huge documentation for safety and quality control, we
picked technologies devoided entirely of these problems.

### Configuration over code

We provide a clean API for configuration.
Don't code what you already wrote too many times. Configure it instead, or use
tailored defaults.

### Hardware integration

Hardware integration for our robots is present right out of the box.
You call the hardware with convinience and clear intent, not with compexity
overhead.

## Technical Specifications

## Use Cases

## Comparison

## Architecture

## Getting Started

## Roadmap

## About
