"use strict";

import React from 'react';
import { TableRow, TableColumn, FontIcon, Button } from 'react-md';
import { Link } from 'react-router-dom';

import { SimpleLink } from './SimpleLink';
import AuthService from '../services/AuthService';
import { withRouter } from 'react-router-dom'


class ConversationListRow extends React.Component {

    constructor(props) {
        super(props);
    }

    componentWillMount(){
        this.user = AuthService.getCurrentUser();
        var username = this.user.username;
        var others_id = undefined;
        this.setState(Object.assign({}, this.state, {username: username}));
        this.other_user = undefined;
        console.log("data",this.props.conversation.messages[0].author.username)
        console.log("data",this.props.conversation.messages[0].recipient.username)
        if (this.props.conversation.messages[0].author.username !== username){
            this.other_user = this.props.conversation.messages[0].author;
        }
        else{
            this.other_user = this.props.conversation.messages[0].recipient;

        }
        var str = this.props.conversation.messages[0].createdAt.split('T');
        var temp = str[0].split('-');
        var date = temp[2].toString() + '-' + temp[1].toString() + '-' + temp[0].toString();
        str[1] = str[1].split('.');
        var time =str[1][0];
        var date_time = time.toString() + " " + date.toString();
        this.props.conversation.messages[0].createdAt = date_time;
    }

    render() {
        console.log("this", this.props.conversation)
        return (
            <div className="row" style={{backgroundColor:"#F0F0F0", border:"0.1px solid"}}>
                <div className="col-2">
                    <div className="row justify-content-md-center" >
                        <Link to={`/profile/${this.other_user._id}`}>
                            <img src= {`http://localhost:3000/photos/${this.other_user.photo}`}  className="message-photo" />
                        </Link>
                    </div>
                    <div className="row justify-content-md-center">
                        <Link to={`/profile/${this.other_user._id}`}>
                            {this.other_user.username}
                        </Link>
                    </div>
                </div>
                <div className="col-8">
                    <div className="row justify-content-md-center">
                        {this.props.conversation.messages[0].body}
                    </div>
                    <div className="row justify-content-md-center">
                        {this.props.conversation.messages[0].createdAt}
                    </div>
                </div>
                <div className="col-2 justify-content-md-center reply">
                    <SimpleLink className= "conv" to={`/conversation/${this.props.conversation.conversationId}`}>Details</SimpleLink>
                </div>
            </div>
        );
    }
}

export default withRouter(ConversationListRow)