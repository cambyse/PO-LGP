<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>AbstractEnvironment.h</name>
    <path>/home/robert/data/git_repos/MLR_git/share/include/MCTS_Environment/</path>
    <filename>AbstractEnvironment_8h</filename>
    <class kind="class">AbstractEnvironment</class>
    <class kind="struct">AbstractEnvironment::Action</class>
    <class kind="struct">AbstractEnvironment::Observation</class>
    <class kind="struct">AbstractEnvironment::State</class>
    <class kind="struct">AbstractEnvironment::ActionHash</class>
    <class kind="struct">AbstractEnvironment::ObservationHash</class>
    <class kind="struct">AbstractEnvironment::ActionEq</class>
    <class kind="struct">AbstractEnvironment::ObservationEq</class>
  </compound>
  <compound kind="file">
    <name>AbstractFiniteEnvironment.h</name>
    <path>/home/robert/data/git_repos/MLR_git/share/include/MCTS_Environment/</path>
    <filename>AbstractFiniteEnvironment_8h</filename>
    <includes id="AbstractEnvironment_8h" name="AbstractEnvironment.h" local="yes" imported="no">AbstractEnvironment.h</includes>
    <class kind="class">AbstractFiniteEnvironment</class>
    <class kind="class">AbstractFiniteEnvironment::TypeWrapper</class>
    <class kind="class">AbstractFiniteEnvironment::FiniteAction</class>
    <class kind="class">AbstractFiniteEnvironment::FiniteState</class>
    <class kind="class">AbstractFiniteEnvironment::FiniteObservation</class>
  </compound>
  <compound kind="file">
    <name>unit_tests.cpp</name>
    <path>/home/robert/data/git_repos/MLR_git/share/include/MCTS_Environment/</path>
    <filename>unit__tests_8cpp</filename>
    <includes id="AbstractFiniteEnvironment_8h" name="AbstractFiniteEnvironment.h" local="yes" imported="no">AbstractFiniteEnvironment.h</includes>
    <class kind="class">TestEnvironment</class>
    <member kind="function">
      <type></type>
      <name>TEST</name>
      <anchorfile>unit__tests_8cpp.html</anchorfile>
      <anchor>ab8a0a8f2c5c0ca91832a931c3f2da26f</anchor>
      <arglist>(AbstractEnvironment, UnorderedSets)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>TEST</name>
      <anchorfile>unit__tests_8cpp.html</anchorfile>
      <anchor>a52cdbe89381638bdd572efad43869297</anchor>
      <arglist>(AbstractEnvironment, OstremOperator)</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractEnvironment</name>
    <filename>classAbstractEnvironment.html</filename>
    <class kind="struct">AbstractEnvironment::Action</class>
    <class kind="struct">AbstractEnvironment::ActionEq</class>
    <class kind="struct">AbstractEnvironment::ActionHash</class>
    <class kind="struct">AbstractEnvironment::Observation</class>
    <class kind="struct">AbstractEnvironment::ObservationEq</class>
    <class kind="struct">AbstractEnvironment::ObservationHash</class>
    <class kind="struct">AbstractEnvironment::State</class>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const Action &gt;</type>
      <name>action_handle_t</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a88ebd676b0c2d7a069ffde1258ea4418</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const Observation &gt;</type>
      <name>observation_handle_t</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a926c83655cc84bd773b437df4b83f765</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::shared_ptr&lt; const State &gt;</type>
      <name>state_handle_t</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>ab42db6af8d2afb7b9b2dc018ba6f2be7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::vector&lt; action_handle_t &gt;</type>
      <name>action_container_t</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a49258f24649987c2244cc8f9cc931eda</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>double</type>
      <name>reward_t</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a4b7d5a3ecee3fe8053029e40dacf68be</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::tuple&lt; observation_handle_t, reward_t &gt;</type>
      <name>observation_reward_pair_t</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a8a9d735588ad8118527460545f4a9bcb</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractEnvironment</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a814f9e6d98b284582ec20725f119a5ba</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~AbstractEnvironment</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>af9d4b4261753376f05ca47b91a64c6b7</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual observation_reward_pair_t</type>
      <name>transition</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a760310067fdfe288f7b61d19ef205972</anchor>
      <arglist>(const action_handle_t &amp;action_handle)=0</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual observation_reward_pair_t</type>
      <name>transition</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a7ef8c50b35698412a8a1c69d12d74461</anchor>
      <arglist>(const state_handle_t &amp;state_handle, const action_handle_t &amp;action_handle)</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual action_container_t</type>
      <name>get_actions</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>ad6b2cecf12c718ba238a4a705df24059</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual state_handle_t</type>
      <name>get_state_handle</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a5f6149279dda72fb0e1c6ba928e77374</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>set_state</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a369bc44859f9c03221cac71b31eee086</anchor>
      <arglist>(const state_handle_t &amp;state_handle)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>has_terminal_state</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a864f2499b45f2e13e8350a5f32302af8</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>is_terminal_state</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>abdddd31c43ed7a2c12ca7d307f454ffb</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>is_terminal_state</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>ae9962bfcf1a238b5876ca1ee1e605828</anchor>
      <arglist>(const state_handle_t &amp;state_handle)</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>is_deterministic</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a882bd8f80bb774094356ad70f97e63c0</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>has_max_reward</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>abe7d6f5e51243019f930432afaa5f347</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual reward_t</type>
      <name>max_reward</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>aaee42f677b361cb5f11e8481a4920739</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>has_min_reward</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a4a891658b8ecd01457630e472906f848</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual reward_t</type>
      <name>min_reward</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>ad8bfbcb9e4c75398933fdd446ed551a0</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>is_markov</name>
      <anchorfile>classAbstractEnvironment.html</anchorfile>
      <anchor>a65bf6c1a894cb269a9223e8b76b40127</anchor>
      <arglist>() const =0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractFiniteEnvironment</name>
    <filename>classAbstractFiniteEnvironment.html</filename>
    <templarg>ACTION</templarg>
    <templarg>STATE</templarg>
    <templarg>OBSERVATION</templarg>
    <base>AbstractEnvironment</base>
    <class kind="class">AbstractFiniteEnvironment::FiniteAction</class>
    <class kind="class">AbstractFiniteEnvironment::FiniteObservation</class>
    <class kind="class">AbstractFiniteEnvironment::FiniteState</class>
    <class kind="class">AbstractFiniteEnvironment::TypeWrapper</class>
    <member kind="typedef">
      <type>FiniteAction</type>
      <name>action_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a50613b82f4b294c311a44e1f09d713b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>FiniteState</type>
      <name>state_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a45b5a1cdd7782ec8b69f01624981ead1</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>FiniteObservation</type>
      <name>observation_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a2b6b855f52361f73880d74aa69345259</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::pair&lt; state_t, reward_t &gt;</type>
      <name>state_reward_pair_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>abaa13241a30c65ab4183f75bd40043d1</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::vector&lt; state_handle_t &gt;</type>
      <name>state_container_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a81dff75d006c8b4058d74ebaf1c87064</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>ac286aa170459486a9180b83524f9fbe2</anchor>
      <arglist>(const std::vector&lt; action_t &gt; action_list, const std::vector&lt; state_t &gt; state_list)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a3a7b4a1b2ebb1e24e71cfa05ad21652f</anchor>
      <arglist>(const std::vector&lt; ACTION &gt; action_list, const std::vector&lt; STATE &gt; state_list)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a4148cd19318b8b02ad88bd25d130c1ce</anchor>
      <arglist>(const std::initializer_list&lt; action_t &gt; action_list, const std::initializer_list&lt; state_t &gt; state_list)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>adb0f47933fa020ab825b4caead89e632</anchor>
      <arglist>(const std::initializer_list&lt; ACTION &gt; action_list, const std::initializer_list&lt; STATE &gt; state_list)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a568153f2fcd27977e69c52cae65edeaa</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual state_reward_pair_t</type>
      <name>finite_transition</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>ad8fb783fc4de868f3838b5066ced7173</anchor>
      <arglist>(const state_t &amp;, const action_t &amp;) const =0</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual observation_reward_pair_t</type>
      <name>transition</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a20138242ba9fc4176e05a377650d292b</anchor>
      <arglist>(const action_handle_t &amp;action_handle) overridefinal</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual action_container_t</type>
      <name>get_actions</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>af4c5e5d38ebc29e8efb455a5686b3f46</anchor>
      <arglist>() overridefinal</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual state_container_t</type>
      <name>get_states</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>aa9ff586fe0873f21bb04bb8d803a22d4</anchor>
      <arglist>() final</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual state_handle_t</type>
      <name>get_state_handle</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>ae4cb802c2551aae26e1cee9273903bde</anchor>
      <arglist>() overridefinal</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>set_state</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a8d38a410e8e8725a6af4deda96252ed5</anchor>
      <arglist>(const state_handle_t &amp;state_handle) overridefinal</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>is_markov</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a496234c9ade4c56983f25ed92a64ae6c</anchor>
      <arglist>() const overridefinal</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::vector&lt; TO &gt;</type>
      <name>convert_vector</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a52ca7531b803f2ece032fe7f6dc78286</anchor>
      <arglist>(const std::vector&lt; FROM &gt; &amp;vec)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static action_container_t</type>
      <name>construct_action_container</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a13f952f19a4611fe6fb5f7c168512b2d</anchor>
      <arglist>(const std::vector&lt; action_t &gt; &amp;action_list)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static state_container_t</type>
      <name>construct_state_container</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a0b5278da3f1c6885cbdd3acaeef94924</anchor>
      <arglist>(const std::vector&lt; state_t &gt; &amp;state_list)</arglist>
    </member>
    <member kind="variable">
      <type>const std::vector&lt; action_t &gt;</type>
      <name>action_list</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a843c4eb367d561fbed2260b77f2399e7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const action_container_t</type>
      <name>action_handle_list</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a55942d41f07a5029c7aa1eaf196289de</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const std::vector&lt; state_t &gt;</type>
      <name>state_list</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a873139b26eec44fc0d62c41d007e2ea7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>state_t</type>
      <name>state</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a039d4bf4e483907086571c1a28d66460</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractFiniteEnvironment&lt; int, int &gt;</name>
    <filename>classAbstractFiniteEnvironment.html</filename>
    <base>AbstractEnvironment</base>
    <member kind="typedef">
      <type>FiniteAction</type>
      <name>action_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a50613b82f4b294c311a44e1f09d713b2</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>FiniteState</type>
      <name>state_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a45b5a1cdd7782ec8b69f01624981ead1</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>FiniteObservation</type>
      <name>observation_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a2b6b855f52361f73880d74aa69345259</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::pair&lt; state_t, reward_t &gt;</type>
      <name>state_reward_pair_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>abaa13241a30c65ab4183f75bd40043d1</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef">
      <type>std::vector&lt; state_handle_t &gt;</type>
      <name>state_container_t</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a81dff75d006c8b4058d74ebaf1c87064</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>ac286aa170459486a9180b83524f9fbe2</anchor>
      <arglist>(const std::vector&lt; action_t &gt; action_list, const std::vector&lt; state_t &gt; state_list)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a3a7b4a1b2ebb1e24e71cfa05ad21652f</anchor>
      <arglist>(const std::vector&lt; int &gt; action_list, const std::vector&lt; int &gt; state_list)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a4148cd19318b8b02ad88bd25d130c1ce</anchor>
      <arglist>(const std::initializer_list&lt; action_t &gt; action_list, const std::initializer_list&lt; state_t &gt; state_list)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>adb0f47933fa020ab825b4caead89e632</anchor>
      <arglist>(const std::initializer_list&lt; int &gt; action_list, const std::initializer_list&lt; int &gt; state_list)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~AbstractFiniteEnvironment</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a568153f2fcd27977e69c52cae65edeaa</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual state_reward_pair_t</type>
      <name>finite_transition</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>ad8fb783fc4de868f3838b5066ced7173</anchor>
      <arglist>(const state_t &amp;, const action_t &amp;) const =0</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual observation_reward_pair_t</type>
      <name>transition</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a20138242ba9fc4176e05a377650d292b</anchor>
      <arglist>(const action_handle_t &amp;action_handle) overridefinal</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual action_container_t</type>
      <name>get_actions</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>af4c5e5d38ebc29e8efb455a5686b3f46</anchor>
      <arglist>() overridefinal</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual state_container_t</type>
      <name>get_states</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>aa9ff586fe0873f21bb04bb8d803a22d4</anchor>
      <arglist>() final</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual state_handle_t</type>
      <name>get_state_handle</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>ae4cb802c2551aae26e1cee9273903bde</anchor>
      <arglist>() overridefinal</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>set_state</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a8d38a410e8e8725a6af4deda96252ed5</anchor>
      <arglist>(const state_handle_t &amp;state_handle) overridefinal</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>is_markov</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a496234c9ade4c56983f25ed92a64ae6c</anchor>
      <arglist>() const overridefinal</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static std::vector&lt; TO &gt;</type>
      <name>convert_vector</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a52ca7531b803f2ece032fe7f6dc78286</anchor>
      <arglist>(const std::vector&lt; FROM &gt; &amp;vec)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static action_container_t</type>
      <name>construct_action_container</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a13f952f19a4611fe6fb5f7c168512b2d</anchor>
      <arglist>(const std::vector&lt; action_t &gt; &amp;action_list)</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static state_container_t</type>
      <name>construct_state_container</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a0b5278da3f1c6885cbdd3acaeef94924</anchor>
      <arglist>(const std::vector&lt; state_t &gt; &amp;state_list)</arglist>
    </member>
    <member kind="variable">
      <type>const std::vector&lt; action_t &gt;</type>
      <name>action_list</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a843c4eb367d561fbed2260b77f2399e7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const action_container_t</type>
      <name>action_handle_list</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a55942d41f07a5029c7aa1eaf196289de</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>const std::vector&lt; state_t &gt;</type>
      <name>state_list</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a873139b26eec44fc0d62c41d007e2ea7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>state_t</type>
      <name>state</name>
      <anchorfile>classAbstractFiniteEnvironment.html</anchorfile>
      <anchor>a039d4bf4e483907086571c1a28d66460</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>AbstractEnvironment::Action</name>
    <filename>structAbstractEnvironment_1_1Action.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Action</name>
      <anchorfile>structAbstractEnvironment_1_1Action.html</anchorfile>
      <anchor>a4f7556f9a895cf99cc94af06dc4773e4</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>operator==</name>
      <anchorfile>structAbstractEnvironment_1_1Action.html</anchorfile>
      <anchor>ab32954256f6c2392d6b8e0df316936df</anchor>
      <arglist>(const Action &amp;other) const =0</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>operator!=</name>
      <anchorfile>structAbstractEnvironment_1_1Action.html</anchorfile>
      <anchor>a3d3fceae328e3972bde1a049bc9458c9</anchor>
      <arglist>(const Action &amp;other) const </arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>get_hash</name>
      <anchorfile>structAbstractEnvironment_1_1Action.html</anchorfile>
      <anchor>ae953762f4304e9c9ba7845d35c9dda65</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>write</name>
      <anchorfile>structAbstractEnvironment_1_1Action.html</anchorfile>
      <anchor>a6e03c6a9f8e155d7e836ccdc8433e734</anchor>
      <arglist>(std::ostream &amp;) const =0</arglist>
    </member>
    <member kind="friend">
      <type>friend std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>structAbstractEnvironment_1_1Action.html</anchorfile>
      <anchor>a68ff081a80959d1146af5a28d09b0373</anchor>
      <arglist>(std::ostream &amp;out, const Action &amp;action)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>AbstractEnvironment::ActionEq</name>
    <filename>structAbstractEnvironment_1_1ActionEq.html</filename>
    <member kind="function">
      <type>size_t</type>
      <name>operator()</name>
      <anchorfile>structAbstractEnvironment_1_1ActionEq.html</anchorfile>
      <anchor>ab4c82980e85380dabadfa2ee4ab4f99d</anchor>
      <arglist>(const action_handle_t &amp;action1, const action_handle_t &amp;action2) const </arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>AbstractEnvironment::ActionHash</name>
    <filename>structAbstractEnvironment_1_1ActionHash.html</filename>
    <member kind="function">
      <type>size_t</type>
      <name>operator()</name>
      <anchorfile>structAbstractEnvironment_1_1ActionHash.html</anchorfile>
      <anchor>a2307a4d30716d31799e0cd9f5aab380a</anchor>
      <arglist>(const action_handle_t &amp;action) const </arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractFiniteEnvironment::FiniteAction</name>
    <filename>classAbstractFiniteEnvironment_1_1FiniteAction.html</filename>
    <base>TypeWrapper&lt; FiniteAction, ACTION &gt;</base>
    <member kind="function">
      <type></type>
      <name>FiniteAction</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteAction.html</anchorfile>
      <anchor>aed5ae0bf995711819c36d7bc5559f9e4</anchor>
      <arglist>(ACTION action)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~FiniteAction</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteAction.html</anchorfile>
      <anchor>a8bfe615286f38f6ec7a8c9d0642a6aaa</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>operator==</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteAction.html</anchorfile>
      <anchor>a64a76aa518ea73e5b185ef2658e6f5c3</anchor>
      <arglist>(const Action &amp;other) const </arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual size_t</type>
      <name>get_hash</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteAction.html</anchorfile>
      <anchor>ad09c6474f2e6b76e8f5d41f468044ee2</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual void</type>
      <name>write</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteAction.html</anchorfile>
      <anchor>a37fa597cbbb079d42b515d9b9840775e</anchor>
      <arglist>(std::ostream &amp;out) const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractFiniteEnvironment::FiniteObservation</name>
    <filename>classAbstractFiniteEnvironment_1_1FiniteObservation.html</filename>
    <base>AbstractEnvironment::Observation</base>
    <base>TypeWrapper&lt; FiniteObservation, OBSERVATION &gt;</base>
    <member kind="function">
      <type></type>
      <name>FiniteObservation</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteObservation.html</anchorfile>
      <anchor>a861307ff2f03ebcd215f30fc4a67a279</anchor>
      <arglist>(OBSERVATION observation)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~FiniteObservation</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteObservation.html</anchorfile>
      <anchor>a235a0c65335108bfe3301dfba6a88b09</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>operator==</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteObservation.html</anchorfile>
      <anchor>aad6c9105e934fff3c97fe1355a7cf5fc</anchor>
      <arglist>(const Observation &amp;other) const </arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual size_t</type>
      <name>get_hash</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteObservation.html</anchorfile>
      <anchor>ae6b068b57854af07cb3d2e36d68ead40</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>write</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteObservation.html</anchorfile>
      <anchor>a4745e8106d29f8d8cc8b7455c7c495e2</anchor>
      <arglist>(std::ostream &amp;out) const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractFiniteEnvironment::FiniteState</name>
    <filename>classAbstractFiniteEnvironment_1_1FiniteState.html</filename>
    <base>AbstractEnvironment::State</base>
    <base>TypeWrapper&lt; FiniteState, STATE &gt;</base>
    <member kind="function">
      <type></type>
      <name>FiniteState</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteState.html</anchorfile>
      <anchor>adac666b02fab9a3e72c871ef2e8fb8b8</anchor>
      <arglist>(STATE state)</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~FiniteState</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteState.html</anchorfile>
      <anchor>af1cef41be996ea8b40c0b1aa0f36262f</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>operator==</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1FiniteState.html</anchorfile>
      <anchor>a77194b3e35a442b83c327a0a962e70a1</anchor>
      <arglist>(const State &amp;other) const </arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>AbstractEnvironment::Observation</name>
    <filename>structAbstractEnvironment_1_1Observation.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~Observation</name>
      <anchorfile>structAbstractEnvironment_1_1Observation.html</anchorfile>
      <anchor>ad01c843c41ded0565151ad8a9c75f2d3</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>operator==</name>
      <anchorfile>structAbstractEnvironment_1_1Observation.html</anchorfile>
      <anchor>a08132a442e62c9ddcc46d9880e29c666</anchor>
      <arglist>(const Observation &amp;other) const =0</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>operator!=</name>
      <anchorfile>structAbstractEnvironment_1_1Observation.html</anchorfile>
      <anchor>a77458ba95a1a2ae8382635fee7b3fcce</anchor>
      <arglist>(const Observation &amp;other) const </arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual size_t</type>
      <name>get_hash</name>
      <anchorfile>structAbstractEnvironment_1_1Observation.html</anchorfile>
      <anchor>aceb49c3983aff0d0217834c4dbd4dd24</anchor>
      <arglist>() const =0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>write</name>
      <anchorfile>structAbstractEnvironment_1_1Observation.html</anchorfile>
      <anchor>a48cec546c8e9cf6405f6a0e5be818f55</anchor>
      <arglist>(std::ostream &amp;) const =0</arglist>
    </member>
    <member kind="friend">
      <type>friend std::ostream &amp;</type>
      <name>operator&lt;&lt;</name>
      <anchorfile>structAbstractEnvironment_1_1Observation.html</anchorfile>
      <anchor>a58663b599c7b7d157fc0c6767577075e</anchor>
      <arglist>(std::ostream &amp;out, const Observation &amp;observation)</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>AbstractEnvironment::ObservationEq</name>
    <filename>structAbstractEnvironment_1_1ObservationEq.html</filename>
    <member kind="function">
      <type>size_t</type>
      <name>operator()</name>
      <anchorfile>structAbstractEnvironment_1_1ObservationEq.html</anchorfile>
      <anchor>abaff44970a4b3c0552aebfcfaa17f22d</anchor>
      <arglist>(const observation_handle_t &amp;observation1, const observation_handle_t &amp;observation2) const </arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>AbstractEnvironment::ObservationHash</name>
    <filename>structAbstractEnvironment_1_1ObservationHash.html</filename>
    <member kind="function">
      <type>size_t</type>
      <name>operator()</name>
      <anchorfile>structAbstractEnvironment_1_1ObservationHash.html</anchorfile>
      <anchor>aabffb3eb506e74b3b47a95284ae0c5be</anchor>
      <arglist>(const observation_handle_t &amp;observation) const </arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>AbstractEnvironment::State</name>
    <filename>structAbstractEnvironment_1_1State.html</filename>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~State</name>
      <anchorfile>structAbstractEnvironment_1_1State.html</anchorfile>
      <anchor>adf172280bfb1fd8afdddc606690ae003</anchor>
      <arglist>()=default</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>TestEnvironment</name>
    <filename>classTestEnvironment.html</filename>
    <base>AbstractFiniteEnvironment&lt; int, int &gt;</base>
    <member kind="function">
      <type></type>
      <name>TestEnvironment</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>a9f7b07b81ef854c739c77da04fd1d0b2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~TestEnvironment</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>a9e394868e925ff7a77d3a10c26931d87</anchor>
      <arglist>()=default</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual state_reward_pair_t</type>
      <name>finite_transition</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>a97582f0d0474fe2a42f4ac2b085bbea5</anchor>
      <arglist>(const state_t &amp;state, const action_t &amp;action) const override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>has_terminal_state</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>ac57c097650d76707ffd8a9f74cf3911e</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>is_terminal_state</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>a5cfc3c01924317f65eacfe77039e2225</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>is_deterministic</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>a725658d657220882b92f3ff8113a4fa5</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>has_max_reward</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>a05d17180478738ab65081489af721c63</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual reward_t</type>
      <name>max_reward</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>a8b230c869b48981f57ab79f0490c5839</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual bool</type>
      <name>has_min_reward</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>acb5d2f2ac84bf2290a6cd7ed9786a66e</anchor>
      <arglist>() const override</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual reward_t</type>
      <name>min_reward</name>
      <anchorfile>classTestEnvironment.html</anchorfile>
      <anchor>a2ceddfc3dae7dfd2b3ca47a57bf1c430</anchor>
      <arglist>() const override</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AbstractFiniteEnvironment::TypeWrapper</name>
    <filename>classAbstractFiniteEnvironment_1_1TypeWrapper.html</filename>
    <templarg>C</templarg>
    <templarg>T</templarg>
    <member kind="typedef">
      <type>T</type>
      <name>value_t</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a8b3a9f4e593d0d6c790cf0ccc760470b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>TypeWrapper</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a1f1a687bc026fbec148e4433404cb8f7</anchor>
      <arglist>(const T &amp;val)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator T</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a0272bf37da7fcec055f505391bdef1cd</anchor>
      <arglist>() const </arglist>
    </member>
    <member kind="function">
      <type>C &amp;</type>
      <name>operator=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a2a725be2b5a9c4cae1c01f2ad5979cc7</anchor>
      <arglist>(const T &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a9a32f87a7c44428a0113f21a9ca535f3</anchor>
      <arglist>(const T &amp;other) const </arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>aec92acbe5bcbfef7e407e5303e8b2bbd</anchor>
      <arglist>(const T &amp;other) const </arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a128b66bab1ae27489b6dbb8402921159</anchor>
      <arglist>(const T &amp;other) const </arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>abb333f8862cf3df2f5bfadecf1ff84a3</anchor>
      <arglist>(const T &amp;other) const </arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>afc5be6cc9c329e86561f48d0ed55a1e6</anchor>
      <arglist>(const T &amp;other) const </arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>aa2c6b6ac9231b48d95671fd2b6e97f7c</anchor>
      <arglist>(const T &amp;other) const </arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>operator+=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a2000ba557afd70351e8c284f8e7f60c6</anchor>
      <arglist>(const T &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>T &amp;</type>
      <name>operator-=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a80d97f7f5d4851bcd605e56c58e080a5</anchor>
      <arglist>(const T &amp;rhs)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>T</type>
      <name>value</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a561c34a33feba058f9a5360a2358de43</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>TypeWrapper&lt; FiniteAction, ACTION &gt;</name>
    <filename>classAbstractFiniteEnvironment_1_1TypeWrapper.html</filename>
    <member kind="typedef">
      <type>ACTION</type>
      <name>value_t</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a8b3a9f4e593d0d6c790cf0ccc760470b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>TypeWrapper</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a1f1a687bc026fbec148e4433404cb8f7</anchor>
      <arglist>(const ACTION &amp;val)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator ACTION</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a0272bf37da7fcec055f505391bdef1cd</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>FiniteAction &amp;</type>
      <name>operator=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a2a725be2b5a9c4cae1c01f2ad5979cc7</anchor>
      <arglist>(const ACTION &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a9a32f87a7c44428a0113f21a9ca535f3</anchor>
      <arglist>(const ACTION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>aec92acbe5bcbfef7e407e5303e8b2bbd</anchor>
      <arglist>(const ACTION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a128b66bab1ae27489b6dbb8402921159</anchor>
      <arglist>(const ACTION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>abb333f8862cf3df2f5bfadecf1ff84a3</anchor>
      <arglist>(const ACTION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>afc5be6cc9c329e86561f48d0ed55a1e6</anchor>
      <arglist>(const ACTION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>aa2c6b6ac9231b48d95671fd2b6e97f7c</anchor>
      <arglist>(const ACTION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>ACTION &amp;</type>
      <name>operator+=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a2000ba557afd70351e8c284f8e7f60c6</anchor>
      <arglist>(const ACTION &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>ACTION &amp;</type>
      <name>operator-=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a80d97f7f5d4851bcd605e56c58e080a5</anchor>
      <arglist>(const ACTION &amp;rhs)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>ACTION</type>
      <name>value</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a561c34a33feba058f9a5360a2358de43</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>TypeWrapper&lt; FiniteObservation, OBSERVATION &gt;</name>
    <filename>classAbstractFiniteEnvironment_1_1TypeWrapper.html</filename>
    <member kind="typedef">
      <type>OBSERVATION</type>
      <name>value_t</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a8b3a9f4e593d0d6c790cf0ccc760470b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>TypeWrapper</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a1f1a687bc026fbec148e4433404cb8f7</anchor>
      <arglist>(const OBSERVATION &amp;val)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator OBSERVATION</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a0272bf37da7fcec055f505391bdef1cd</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>FiniteObservation &amp;</type>
      <name>operator=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a2a725be2b5a9c4cae1c01f2ad5979cc7</anchor>
      <arglist>(const OBSERVATION &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a9a32f87a7c44428a0113f21a9ca535f3</anchor>
      <arglist>(const OBSERVATION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>aec92acbe5bcbfef7e407e5303e8b2bbd</anchor>
      <arglist>(const OBSERVATION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a128b66bab1ae27489b6dbb8402921159</anchor>
      <arglist>(const OBSERVATION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>abb333f8862cf3df2f5bfadecf1ff84a3</anchor>
      <arglist>(const OBSERVATION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>afc5be6cc9c329e86561f48d0ed55a1e6</anchor>
      <arglist>(const OBSERVATION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>aa2c6b6ac9231b48d95671fd2b6e97f7c</anchor>
      <arglist>(const OBSERVATION &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>OBSERVATION &amp;</type>
      <name>operator+=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a2000ba557afd70351e8c284f8e7f60c6</anchor>
      <arglist>(const OBSERVATION &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>OBSERVATION &amp;</type>
      <name>operator-=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a80d97f7f5d4851bcd605e56c58e080a5</anchor>
      <arglist>(const OBSERVATION &amp;rhs)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>OBSERVATION</type>
      <name>value</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a561c34a33feba058f9a5360a2358de43</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>TypeWrapper&lt; FiniteState, STATE &gt;</name>
    <filename>classAbstractFiniteEnvironment_1_1TypeWrapper.html</filename>
    <member kind="typedef">
      <type>STATE</type>
      <name>value_t</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a8b3a9f4e593d0d6c790cf0ccc760470b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>TypeWrapper</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a1f1a687bc026fbec148e4433404cb8f7</anchor>
      <arglist>(const STATE &amp;val)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>operator STATE</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a0272bf37da7fcec055f505391bdef1cd</anchor>
      <arglist>() const</arglist>
    </member>
    <member kind="function">
      <type>FiniteState &amp;</type>
      <name>operator=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a2a725be2b5a9c4cae1c01f2ad5979cc7</anchor>
      <arglist>(const STATE &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator==</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a9a32f87a7c44428a0113f21a9ca535f3</anchor>
      <arglist>(const STATE &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator!=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>aec92acbe5bcbfef7e407e5303e8b2bbd</anchor>
      <arglist>(const STATE &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a128b66bab1ae27489b6dbb8402921159</anchor>
      <arglist>(const STATE &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>abb333f8862cf3df2f5bfadecf1ff84a3</anchor>
      <arglist>(const STATE &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&lt;=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>afc5be6cc9c329e86561f48d0ed55a1e6</anchor>
      <arglist>(const STATE &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>operator&gt;=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>aa2c6b6ac9231b48d95671fd2b6e97f7c</anchor>
      <arglist>(const STATE &amp;other) const</arglist>
    </member>
    <member kind="function">
      <type>STATE &amp;</type>
      <name>operator+=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a2000ba557afd70351e8c284f8e7f60c6</anchor>
      <arglist>(const STATE &amp;rhs)</arglist>
    </member>
    <member kind="function">
      <type>STATE &amp;</type>
      <name>operator-=</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a80d97f7f5d4851bcd605e56c58e080a5</anchor>
      <arglist>(const STATE &amp;rhs)</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>STATE</type>
      <name>value</name>
      <anchorfile>classAbstractFiniteEnvironment_1_1TypeWrapper.html</anchorfile>
      <anchor>a561c34a33feba058f9a5360a2358de43</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>std</name>
    <filename>namespacestd.html</filename>
  </compound>
</tagfile>
