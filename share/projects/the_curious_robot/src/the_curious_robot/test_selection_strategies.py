#!/usr/bin/env python
# encoding: utf-8

from sas_pick_ooi import StrategySelectEntropy


class TestEntropyMaxStrategy(object):
    def test_max_entropy_single_entry(self):
        strategy = StrategySelectEntropy("max")

        entropies = {0: [-.2]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 0

        entropies = {0: [-.2], 1: [-.4]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 0

        entropies = {0: [-.2], 1: [-.1]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 1

    def test_max_entropy_multiple_entries(self):
        strategy = StrategySelectEntropy("max")

        entropies = {0: [-.2, -3]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 0

        entropies = {0: [-.2, -.2], 1: [-.4]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 0

        entropies = {0: [-.2, -.2],
                     1: [-.1]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 1

        entropies = {0: [-.8, -.8, -.1],
                     1: [-.2]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 0


class TestEntropyMeanStrategy(object):
    def test_(self):
        strategy = StrategySelectEntropy("mean")

        entropies = {0: [-.2]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 0

        entropies = {0: [-.2], 1: [-.25, -.1]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 1

        entropies = {0: [-.2], 1: [-.35, -.1]}
        selected = strategy.execute(oois=None, entropies=entropies)
        assert selected == 0
