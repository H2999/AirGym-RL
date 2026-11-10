"""The previous reference-trajectory initializer is retired.

Current armor_impact learns recovery from PPO rewards alone. Existing legacy
checkpoints and reports are retained as historical results, not new-task models.
"""
if __name__ == '__main__':
    raise SystemExit(
        'Reference initialization is disabled. Run scripts/train_armor_impact.py '
        'without --checkpoint to train the 30-observation task from scratch.'
    )
