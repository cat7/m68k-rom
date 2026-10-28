==========================================
More 68k Macs for qemu-system-m68k (q700)
==========================================

This ``q700`` branch is a personal fork of QEMU. It adds seven 68k
Macintosh models to ``qemu-system-m68k``, next to the Quadra 800 that QEMU
already has. It is an experiment and is not meant for upstream QEMU.

Machines
========

============  ======================  =========  ==============================
Machine       Model                   CPU        ROM
============  ======================  =========  ==============================
``macii``     Macintosh II            68020      Mac II or IIx ROM
``maciix``    Macintosh IIx           68030      IIx ROM
``maciicx``   Macintosh IIcx          68030      IIx ROM
``maciifx``   Macintosh IIfx          68030      IIfx ROM
``q700``      Macintosh Quadra 700    68040      Quadra 700/900 ROM
``q900``      Macintosh Quadra 900    68040      Quadra 700/900 ROM
``lc475``     Macintosh LC 475        68LC040    LC 475 ROM
``q800``      Macintosh Quadra 800    68040      Quadra 800 ROM (upstream)
============  ======================  =========  ==============================

All of them boot System 7 to the Finder from SCSI disks, CD-ROMs and
floppies. A/UX 3.0.1 runs on the Quadra 700.

What the branch adds
====================

- **CPU:** the 68030's on-chip PMMU; 68020/030 bus errors.
- **SCSI:** an NCR 5380 controller for the Mac II family, the IIfx DMA
  chip, and the Quadra 900's second (external) bus. SCSI disks and CDs
  report Apple drive identities, so Apple HD SC Setup accepts them.
- **Video:** the Quadra 700/900 DAFB with all its Apple monitors (12",
  13"/VGA, 15" portrait, 16" and 21"). For the Mac II family there is a
  NuBus video card, ``nubus-video``, with its own declaration ROM and
  driver. It sits in slot 9 by default. The Quadra 800 has its real 1 MB
  of video memory, so 832 x 624 offers thousands of colours, not millions.
  In every depth the gamma table chosen in Monitors applies, as on the
  real machines.
- **Floppies:** the original SWIM, the IWM mode for the Mac II family, and
  the IIfx/Quadra 900 SWIM IOP. Supported images are 400K and 800K (GCR)
  and 720K and 1.44 MB (MFM). They can be read, written and formatted
  from the Mac. Choosing Two-Sided when formatting a 400K image grows it
  to 800K.
- **Ethernet:** an Apple NuBus Ethernet card, ``nubus-ethernet``, for the
  Mac II family, and the LC 475 PDS Ethernet card, ``mac-pds-ethernet``.
- **Shared folder:** a host folder as a Mac disk, through the
  ``nubus-virtio-mmio`` card, ``virtio-9p-device`` and
  ``-fsdev local,...,macos_compat=on`` (file types and resource forks).
  The card's declaration ROM and drivers come from
  https://github.com/cat7/classicvirtio. The LC 475 has only its PDS
  slot, so ``mac-pds-ethernet-virtio`` puts the Ethernet card and the
  virtio devices on one card (ROM: classicvirtio's ``declrom-pds``).
- **Other:** the Egret, Cuda and OSS chips, and the IOP for ADB.

Using it
========

Apple ROMs are not included; you need your own ROM image. The Mac ROMs
time themselves at power-on, so always run with
``-icount shift=6,align=on``. For example::

  qemu-system-m68k -M q700 -m 64 -bios Quadra700.rom \
      -icount shift=6,align=on -g 832x624 \
      -drive file=system7.img,format=raw,media=disk,if=none,id=hd0 \
      -device scsi-hd,drive=hd0,scsi-id=0

A Mac-style launcher that sets all of this up, and can change floppies
while a machine runs, is at https://github.com/cat7/Qemu-GUI-m68k.

The rest of this file is QEMU's own README.

===========
QEMU README
===========

QEMU is a generic and open source machine & userspace emulator and
virtualizer.

QEMU is capable of emulating a complete machine in software without any
need for hardware virtualization support. By using dynamic translation,
it achieves very good performance. QEMU can also integrate with the Xen
and KVM hypervisors to provide emulated hardware while allowing the
hypervisor to manage the CPU. With hypervisor support, QEMU can achieve
near native performance for CPUs. When QEMU emulates CPUs directly it is
capable of running operating systems made for one machine (e.g. an ARMv7
board) on a different machine (e.g. an x86_64 PC board).

QEMU is also capable of providing userspace API virtualization for Linux
and BSD kernel interfaces. This allows binaries compiled against one
architecture ABI (e.g. the Linux PPC64 ABI) to be run on a host using a
different architecture ABI (e.g. the Linux x86_64 ABI). This does not
involve any hardware emulation, simply CPU and syscall emulation.

QEMU aims to fit into a variety of use cases. It can be invoked directly
by users wishing to have full control over its behaviour and settings.
It also aims to facilitate integration into higher level management
layers, by providing a stable command line interface and monitor API.
It is commonly invoked indirectly via the libvirt library when using
open source applications such as oVirt, OpenStack and virt-manager.

QEMU as a whole is released under the GNU General Public License,
version 2. For full licensing details, consult the LICENSE file.


Documentation
=============

Documentation can be found hosted online at
`<https://www.qemu.org/documentation/>`_. The documentation for the
current development version that is available at
`<https://www.qemu.org/docs/master/>`_ is generated from the ``docs/``
folder in the source tree, and is built by `Sphinx
<https://www.sphinx-doc.org/en/master/>`_.


Building
========

QEMU is multi-platform software intended to be buildable on all modern
Linux platforms, OS-X, Win32 (via the Mingw64 toolchain) and a variety
of other UNIX targets. The simple steps to build QEMU are:


.. code-block:: shell

  mkdir build
  cd build
  ../configure
  make

Additional information can also be found online via the QEMU website:

* `<https://wiki.qemu.org/Hosts/Linux>`_
* `<https://wiki.qemu.org/Hosts/Mac>`_
* `<https://wiki.qemu.org/Hosts/W32>`_


Submitting patches
==================

The QEMU source code is maintained under the GIT version control system.

.. code-block:: shell

   git clone https://gitlab.com/qemu-project/qemu.git

When submitting patches, one common approach is to use 'git
format-patch' and/or 'git send-email' to format & send the mail to the
qemu-devel@nongnu.org mailing list. All patches submitted must contain
a 'Signed-off-by' line from the author. Patches should follow the
guidelines set out in the `style section
<https://www.qemu.org/docs/master/devel/style.html>`_ of
the Developers Guide.

Additional information on submitting patches can be found online via
the QEMU website:

* `<https://wiki.qemu.org/Contribute/SubmitAPatch>`_
* `<https://wiki.qemu.org/Contribute/TrivialPatches>`_

The QEMU website is also maintained under source control.

.. code-block:: shell

  git clone https://gitlab.com/qemu-project/qemu-web.git

* `<https://www.qemu.org/2017/02/04/the-new-qemu-website-is-up/>`_

A 'git-publish' utility was created to make above process less
cumbersome, and is highly recommended for making regular contributions,
or even just for sending consecutive patch series revisions. It also
requires a working 'git send-email' setup, and by default doesn't
automate everything, so you may want to go through the above steps
manually for once.

For installation instructions, please go to:

*  `<https://github.com/stefanha/git-publish>`_

The workflow with 'git-publish' is:

.. code-block:: shell

  $ git checkout master -b my-feature
  $ # work on new commits, add your 'Signed-off-by' lines to each
  $ git publish

Your patch series will be sent and tagged as my-feature-v1 if you need to refer
back to it in the future.

Sending v2:

.. code-block:: shell

  $ git checkout my-feature # same topic branch
  $ # making changes to the commits (using 'git rebase', for example)
  $ git publish

Your patch series will be sent with 'v2' tag in the subject and the git tip
will be tagged as my-feature-v2.

Bug reporting
=============

The QEMU project uses GitLab issues to track bugs. Bugs
found when running code built from QEMU git or upstream released sources
should be reported via:

* `<https://gitlab.com/qemu-project/qemu/-/issues>`_

If using QEMU via an operating system vendor pre-built binary package, it
is preferable to report bugs to the vendor's own bug tracker first. If
the bug is also known to affect latest upstream code, it can also be
reported via GitLab.

For additional information on bug reporting consult:

* `<https://wiki.qemu.org/Contribute/ReportABug>`_


ChangeLog
=========

For version history and release notes, please visit
`<https://wiki.qemu.org/ChangeLog/>`_ or look at the git history for
more detailed information.


Contact
=======

The QEMU community can be contacted in a number of ways, with the two
main methods being email and IRC:

* `<mailto:qemu-devel@nongnu.org>`_
* `<https://lists.nongnu.org/mailman/listinfo/qemu-devel>`_
* #qemu on irc.oftc.net

Information on additional methods of contacting the community can be
found online via the QEMU website:

* `<https://wiki.qemu.org/Contribute/StartHere>`_
