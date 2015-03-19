#ifndef BACKUPMETHOD_H_
#define BACKUPMETHOD_H_

namespace backup_method {

    /**
     * Abstract basis class for backup methods. For each internal node that was
     * visited a backup will be performed. This usually are either Monte-Carlo
     * backups using rollouts from that node or dynamic programming backups. */
    class BackupMethod {
        //----typedefs/classes----//

        //----members----//
        //----methods----//
    public:
    BackupMethod() {}
    };

} // namespace backup_method {

#endif /* BACKUPMETHOD_H_ */
