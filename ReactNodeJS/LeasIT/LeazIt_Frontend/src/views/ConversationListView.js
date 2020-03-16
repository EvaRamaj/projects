"use strict";

import React from 'react';

import ConversationList  from '../components/ConversationList';

import ConversationService from '../services/ConversationService';

export class ConversationListView extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            loading: false,
            data: []
        };
    }

    componentWillMount(){
        ConversationService.getConversations().then((data) => {
            console.log("data10",data)
            this.setState({
                data: [...data.conversations],
                loading: false,
                users: [],
            });
        }).catch((e) => {
            console.error(e);
        });
    }


    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }
        return (
            <ConversationList data={this.state.data} />
        );
    }
}
